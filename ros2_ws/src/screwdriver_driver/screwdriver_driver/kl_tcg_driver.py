#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# ============================================================================
#  kl_tcg_driver.py  --  Driver ROS2 (Level 1) pour le controleur KILEWS KL-TCG
# ============================================================================
#
#  ROLE DE CE NODE (perimetre "driver pur") :
#    - LIRE le flux serie RS232 pousse par le controleur KL-TCG.
#    - DECODER les trames (REQ100, DATA100, DATA101) et les republier sur des
#      topics ROS2.
#    - REPONDRE la synchro CMD100 (acquittement + heure) quand le controleur
#      le demande.
#
#  CE QUE CE NODE NE FAIT PAS (et ne doit jamais faire) :
#    - Il ne commande AUCUN vissage. Le manuel KILEWS (page 13, VER:2020060201)
#      est formel : le seul message PC -> controleur est CMD100, qui sert
#      uniquement a synchroniser l'heure et a acquitter. Aucune trame "start"
#      n'existe en RS232. Le declenchement d'un vissage passe par les
#      entrees numeriques (I/O) du controleur, pilotees cote robot/cellule.
#      => Cette logique appartient a l'orchestrateur, PAS a ce driver.
#
#  ----------------------------------------------------------------------------
#  GRANDEURS DISPONIBLES EN SERIE (validees sur trames reelles)
#  ----------------------------------------------------------------------------
#  DATA101 (live, pendant le vissage) : temps + COUPLE live + VITESSE.
#       Le COUPLE LIVE est disponible (champ [2]). Il reste a ~0 pendant la
#       rotation libre (rien ne resiste) et monte au contact resistant
#       (serrage ou blocage). Exemple valide : champ [2]=0.301 N.m juste avant
#       un statut NGQ, identique a la valeur lue a l'ecran du controleur.
#       => Une analyse de signature couple-temps EN LIGNE est donc possible.
#  DATA100 (resultat final)           : COUPLE final + temps + ANGLE + statut.
#
#  NIVEAU DE CERTITUDE DES CHAMPS :
#     [MANUEL]      = documente dans le manuel KL-TCG.
#     [VALIDE]      = confirme par trames reelles + recoupement.
#     [OBSERVE]     = deduit de trames reelles, fiable mais non recoupe.
#     [A CONFIRMER] = hypothese non verifiee.
# ============================================================================

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32

import serial
from datetime import datetime


# ----------------------------------------------------------------------------
#  [MANUEL] keycode = checksum + 5438. Constante issue de la spec KILEWS.
# ----------------------------------------------------------------------------
KEYCODE_OFFSET = 5438


class KLTcgDriver(Node):
    def __init__(self):
        super().__init__("kl_tcg_driver")

        # --- Parametres ROS2 ---------------------------------------------
        self.declare_parameter("port", "/dev/ttyUSB0")
        self.declare_parameter("baudrate", 115200)
        # enable_sync : si True, le node repond CMD100. Indispensable pour que
        # le controleur cesse de repeter DATA100 en boucle (il attend l'acquit).
        self.declare_parameter("enable_sync", True)

        # --- Publishers ---------------------------------------------------
        # raw_data       : trame brute, telle que recue (tracabilite)
        # device_status  : statut controleur decode (depuis REQ100)
        # live_torque    : COUPLE live en N.m (depuis DATA101 champ [2])
        # live_rpm       : VITESSE de rotation live en tr/min (DATA101 champ [3])
        # final_torque   : COUPLE final en N.m (depuis DATA100 champ [19])
        # final_result   : resultat final complet, lisible (depuis DATA100)
        # human_status   : message lisible par un humain (logs/diagnostic)
        self.raw_pub = self.create_publisher(String, "/screwdriver/raw_data", 10)
        self.device_status_pub = self.create_publisher(String, "/screwdriver/device_status", 10)
        self.live_torque_pub = self.create_publisher(Float32, "/screwdriver/live_torque", 10)
        self.live_rpm_pub = self.create_publisher(Float32, "/screwdriver/live_rpm", 10)
        self.final_torque_pub = self.create_publisher(Float32, "/screwdriver/final_torque", 10)
        self.final_result_pub = self.create_publisher(String, "/screwdriver/final_result", 10)
        self.human_status_pub = self.create_publisher(String, "/screwdriver/human_status", 10)

        # --- Lecture des parametres ---------------------------------------
        port = self.get_parameter("port").value
        baudrate = self.get_parameter("baudrate").value
        self.enable_sync = self.get_parameter("enable_sync").value

        # --- Ouverture du port serie (config 8N1) -------------------------
        try:
            self.ser = serial.Serial(
                port=port,
                baudrate=baudrate,
                bytesize=8,
                parity="N",
                stopbits=1,
                timeout=0.1,
            )
            self.get_logger().info(f"Connected to KL-TCG on {port} at {baudrate}")
        except Exception as e:
            self.get_logger().error(f"Could not open serial port: {e}")
            self.ser = None

        # --- Buffer d'assemblage des trames -------------------------------
        # On n'utilise PAS readline() (qui coupe/colle mal les trames).
        # On accumule les octets et on extrait les trames delimitees par { }.
        self.buffer = ""

        # --- Timer de lecture (toutes les 20 ms) --------------------------
        self.timer = self.create_timer(0.02, self.read_serial)

    # ====================================================================
    #  BOUCLE DE LECTURE
    # ====================================================================
    def read_serial(self):
        if self.ser is None:
            return
        try:
            chunk = self.ser.read(4096)
            if not chunk:
                return
            self.buffer += chunk.decode(errors="ignore")
            for frame in self.extract_frames():
                self.process_frame(frame)
        except Exception as e:
            self.get_logger().error(f"Serial read error: {e}")

    def extract_frames(self):
        """
        Decoupe self.buffer en trames completes { ... }.
        - Le bruit avant la 1ere '{' est jete.
        - Chaque '{...}' complet est renvoye.
        - Une '{' ouverte sans '}' est conservee pour le prochain passage.
        """
        frames = []
        while True:
            start = self.buffer.find("{")
            if start == -1:
                self.buffer = ""
                break
            end = self.buffer.find("}", start)
            if end == -1:
                self.buffer = self.buffer[start:]
                break
            frames.append(self.buffer[start:end + 1])
            self.buffer = self.buffer[end + 1:]
        return frames

    def process_frame(self, frame):
        """Aiguille une trame complete vers le bon decodeur."""
        self.publish_string(self.raw_pub, frame)

        fields = self.parse_frame(frame)
        if not fields:
            return

        command = fields[0]
        if command == "REQ100":
            self.handle_req100(fields, frame)
        elif command == "DATA101":
            self.handle_data101(fields, frame)
        elif command == "DATA100":
            self.handle_data100(fields, frame)
        else:
            self.publish_string(
                self.human_status_pub,
                f"Unknown KL-TCG frame ({command}): {frame}"
            )

    def parse_frame(self, frame):
        """Retire les accolades et decoupe sur les virgules."""
        clean = frame.strip().strip("{}")
        if not clean:
            return []
        return clean.split(",")

    @staticmethod
    def interpret_status(status):
        """
        Traduit le code statut brut en texte explicite.
        [OBSERVE] Codes rencontres :
          - contient "OK"  : vissage reussi.
          - "1NG-F"        : NG type F. Couple cible non atteint ; la vis a
                             tourne (temps/angle non nuls, couple final=0).
                             Typique d'un devissage ou d'une vis "en l'air".
          - "1NGQ_"        : NG type Q (torQue). Couple non nul mais rotation
                             quasi nulle. Typique d'un blocage.
        [A CONFIRMER] La signification exacte des suffixes F / Q n'est pas
        documentee ; deduite du comportement des champs.
        """
        if "OK" in status:
            return "OK (success)"
        if "NGQ" in status:
            return "NGQ (failure: torque reached without rotation, likely jam)"
        if "NG-F" in status:
            return "NG-F (failure: rotated without reaching target torque)"
        if "NG" in status:
            return f"NG (failure, code {status})"
        return f"unknown (code {status})"

    # ====================================================================
    #  DECODEUR REQ100  (statut periodique du controleur)
    # ====================================================================
    #  [MANUEL] Au demarrage et au repos, le controleur envoie {REQ100,...}
    #  chaque seconde. L'appareil externe DOIT repondre {CMD100,...}.
    #  [OBSERVE] Le mapping ci-dessous fonctionne sur trames REQ100 reelles.
    # ====================================================================
    def handle_req100(self, fields, raw_line):
        # La synchro est la raison d'etre de REQ100 : on repond d'abord.
        self.send_cmd100(fields)

        try:
            job = fields[16]
            sequence = fields[17]
            program = fields[19]
            tool_connected = fields[21]
            tool_enabled = fields[24]
            tool_stop_status = fields[25]
            screw_count = fields[26]

            tool_connected_text = "tool connected" if tool_connected == "1" else "tool not connected"
            tool_enabled_text = "tool enabled" if tool_enabled == "1" else "tool disabled"

            decoded = (
                f"Controller status (REQ100): controller alive | "
                f"{tool_connected_text} | {tool_enabled_text} | "
                f"job={job} | sequence={sequence} | program={program} | "
                f"screw_count={screw_count} | stop_status={tool_stop_status}"
            )
            self.publish_string(self.device_status_pub, decoded)
            self.publish_string(self.human_status_pub, decoded)
        except IndexError:
            self.publish_string(
                self.human_status_pub,
                f"Incomplete REQ100 frame: {raw_line}"
            )

    # ====================================================================
    #  DECODEUR DATA101  (donnee live pendant le vissage)
    # ====================================================================
    #  FORMAT REEL : {DATA101, temps, couple, vitesse, }
    #                  index:    0     1      2       3     4(vide)
    #
    #  [VALIDE] index[1] = temps de vissage en secondes (monte 0 -> ~10 s).
    #  [VALIDE] index[2] = COUPLE live en N.m. Reste ~0 en rotation libre,
    #            monte au contact resistant. Exemple : 0.301 juste avant NGQ,
    #            identique a l'ecran du controleur.
    #  [VALIDE] index[3] = VITESSE de rotation en tr/min. Profil typique
    #            d'une vitesse (montee rapide puis plateau ~1600).
    # ====================================================================
    def handle_data101(self, fields, raw_line):
        try:
            # Trames de transition tronquees (3 champs) : ignorees en silence.
            if len(fields) < 4:
                return

            fastening_time_s = float(fields[1])   # [VALIDE] temps (s)
            torque_nm = float(fields[2])          # [VALIDE] couple live (N.m)
            speed_rpm = float(fields[3])          # [VALIDE] vitesse (tr/min)

            torque_msg = Float32()
            torque_msg.data = torque_nm
            self.live_torque_pub.publish(torque_msg)

            rpm_msg = Float32()
            rpm_msg.data = speed_rpm
            self.live_rpm_pub.publish(rpm_msg)

            decoded = (
                f"Live data (DATA101): time={fastening_time_s:.3f} s | "
                f"torque={torque_nm:.3f} N.m | speed={speed_rpm:.0f} rpm"
            )
            self.publish_string(self.human_status_pub, decoded)
        except (IndexError, ValueError):
            self.publish_string(
                self.human_status_pub,
                f"Could not decode DATA101 frame: {raw_line}"
            )

    # ====================================================================
    #  DECODEUR DATA100  (resultat final apres chaque vissage)
    # ====================================================================
    #  MAPPING REEL (index Python, trame a 29 champs avec vide final) :
    #     [0]  DATA100        type de trame
    #     [1..6] horodatage (annee/mois/jour/heure/min/sec)
    #     [11] T01VE00631...  [OBSERVE] Tool ID
    #     [12] C14Z-E02184... [OBSERVE] Numero de serie
    #     [13] compteur total [MANUEL] nb total de vissages controleur.
    #                         Distingue un nouveau resultat d'une repetition.
    #     [16] 01             [OBSERVE] numero de programme
    #     [17] ******         [OBSERVE] nom de programme
    #     [18] 01             [OBSERVE] outil selectionne
    #     [19] 0000.3010      [VALIDE] COUPLE final (N.m). Non nul quand le
    #                         couple est atteint (NGQ/OK), 0 en NG-F.
    #     [20] 1              [A CONFIRMER] role inconnu.
    #     [21] 0000.0880      [VALIDE] TEMPS de vissage (s).
    #     [22] 0000.7         [VALIDE] ANGLE parcouru (tours).
    #     [23] 05/05          [OBSERVE] compteur de vis (faites/total).
    #     [25] 1NG-F / 1NGQ_  [OBSERVE] STATUT (voir interpret_status).
    # ====================================================================
    def handle_data100(self, fields, raw_line):
        # Repondre CMD100 : sinon le controleur repete DATA100 en boucle.
        self.send_cmd100(fields)

        try:
            total_count = fields[13]      # [MANUEL] compteur total controleur
            program = fields[16]          # [OBSERVE] numero de programme
            program_name = fields[17]     # [OBSERVE] nom de programme
            selected_tool = fields[18]    # [OBSERVE] outil selectionne
            torque_nm = float(fields[19]) # [VALIDE] couple final (N.m)
            fastening_time_s = fields[21] # [VALIDE] temps de vissage (s)
            angle_turns = fields[22]      # [VALIDE] angle parcouru (tours)
            screw_count = fields[23]      # [OBSERVE] vis faites/total
            status_text = self.interpret_status(fields[25])

            torque_msg = Float32()
            torque_msg.data = torque_nm
            self.final_torque_pub.publish(torque_msg)

            decoded = (
                f"Final result (DATA100): status={status_text} | "
                f"torque={torque_nm:.4f} N.m | "
                f"fastening_time={fastening_time_s} s | "
                f"angle={angle_turns} turns | "
                f"screw_count={screw_count} | "
                f"program={program} | program_name={program_name} | "
                f"tool={selected_tool} | controller_total={total_count}"
            )
            self.publish_string(self.final_result_pub, decoded)
            self.publish_string(self.human_status_pub, decoded)
        except (IndexError, ValueError):
            self.publish_string(
                self.human_status_pub,
                f"Incomplete or invalid DATA100 frame: {raw_line}"
            )

    # ====================================================================
    #  SYNCHRO CMD100  (la SEULE ecriture serie autorisee)
    # ====================================================================
    #  [MANUEL] Format (page 13) :
    #    {CMD100,YEAR,MONTH,DAY,HOUR,MINUTE,SECOND,CHECKSUM,KEYCODE,0,INSTR}
    #    - CHECKSUM = YEAR+MONTH+DAY+HOUR+MINUTE+SECOND
    #    - KEYCODE  = CHECKSUM + 5438
    #    - champ 10 = 0 (defaut)
    #    - INSTR    = instruction number (recopie de la trame entrante).
    #
    #  Ceci n'est PAS une commande de vissage : uniquement acquittement +
    #  mise a l'heure. Aucune action mecanique declenchee.
    #
    #  [A CONFIRMER] Position exacte de "instruction number" a affiner.
    # ====================================================================
    def send_cmd100(self, fields):
        if not self.enable_sync or self.ser is None:
            return
        try:
            now = datetime.now()
            checksum = now.year + now.month + now.day + now.hour + now.minute + now.second
            keycode = checksum + KEYCODE_OFFSET

            instr = "1"
            for v in reversed(fields):
                if v.strip() != "":
                    instr = v.strip()
                    break

            cmd = (
                f"{{CMD100,{now.year:04d},{now.month:02d},{now.day:02d},"
                f"{now.hour:02d},{now.minute:02d},{now.second:02d},"
                f"{checksum:04d},{keycode:04d},0,{instr}}}"
            )
            self.ser.write((cmd + "\r\n").encode())
            self.get_logger().info(f"Sent sync CMD100: {cmd}")
        except Exception as e:
            self.get_logger().error(f"Could not send CMD100: {e}")

    # ====================================================================
    #  UTILITAIRE de publication String + log
    # ====================================================================
    def publish_string(self, publisher, text):
        msg = String()
        msg.data = text
        publisher.publish(msg)
        self.get_logger().info(text)


def main(args=None):
    rclpy.init(args=args)
    node = KLTcgDriver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()