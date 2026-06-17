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
#  NOTE SUR LE NIVEAU DE CERTITUDE DES CHAMPS :
#    Les commentaires distinguent :
#      [MANUEL]   = documente officiellement dans le manuel KL-TCG.
#      [OBSERVE]  = deduit de l'analyse de trames reelles capturees (fiable
#                   mais a confirmer si le firmware/config change).
#      [A CONFIRMER] = hypothese non verifiee, a valider plus tard.
# ============================================================================

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32

import serial
from datetime import datetime


# ----------------------------------------------------------------------------
#  Table des unites de couple.
#  [MANUEL] Le code d'unite figure dans la trame DATA100. Mapping issu du
#  manuel KILEWS. "1" = N.m, qui est l'unite attendue pour ce visseur.
# ----------------------------------------------------------------------------
TORQUE_UNITS = {
    "0": "kgf.cm",
    "1": "N.m",
    "2": "lbf.in",
    "3": "kgf.m",
}

# ----------------------------------------------------------------------------
#  Constante du protocole CMD100.
#  [MANUEL] keycode = checksum + 5438. Cette constante "magique" vient
#  directement de la specification KILEWS, on ne la choisit pas.
# ----------------------------------------------------------------------------
KEYCODE_OFFSET = 5438


class KLTcgDriver(Node):
    def __init__(self):
        super().__init__("kl_tcg_driver")

        # --- Parametres ROS2 (surchargeables au lancement) ---------------
        # Port serie. /dev/ttyUSB0 est le nom typique de l'adaptateur
        # USB<->RS232 sous Linux. A adapter si le systeme l'enumere autrement.
        self.declare_parameter("port", "/dev/ttyUSB0")
        # Baudrate. 115200 = valeur supposee. Si les trames sortent en
        # charabia, c'est le premier parametre a verifier (cf. menu du
        # controleur ou manuel).
        self.declare_parameter("baudrate", 115200)
        # Active ou non l'envoi de la synchro CMD100. Mis a True pour que le
        # controleur cesse de repeter DATA100 en boucle (il attend l'acquit).
        self.declare_parameter("enable_sync", True)

        # --- Publishers (inchanges par rapport a la version d'origine) ----
        # raw_data        : la trame brute, telle que recue (debug/tracabilite)
        # device_status   : statut controleur decode (depuis REQ100)
        # live_torque     : la valeur live variable pendant le vissage (DATA101)
        # final_result    : resultat final decode (depuis DATA100)
        # human_status    : message lisible par un humain (logs/diagnostic)
        self.raw_pub = self.create_publisher(String, "/screwdriver/raw_data", 10)
        self.device_status_pub = self.create_publisher(String, "/screwdriver/device_status", 10)
        self.live_torque_pub = self.create_publisher(Float32, "/screwdriver/live_torque", 10)
        self.final_result_pub = self.create_publisher(String, "/screwdriver/final_result", 10)
        self.human_status_pub = self.create_publisher(String, "/screwdriver/human_status", 10)

        # --- Lecture des parametres ---------------------------------------
        port = self.get_parameter("port").value
        baudrate = self.get_parameter("baudrate").value
        self.enable_sync = self.get_parameter("enable_sync").value

        # --- Ouverture du port serie --------------------------------------
        # bytesize=8, parity="N", stopbits=1 => config "8N1", standard.
        # timeout=0.1 : la lecture rend la main au bout de 0.1 s meme sans
        # donnee, ce qui evite de bloquer le node.
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
        # POURQUOI un buffer ? L'ancienne version utilisait readline(), qui
        # decoupe sur les retours a la ligne. Probleme observe dans les
        # captures : des trames arrivent collees ou coupees (ex: un morceau
        # de DATA101 colle a une autre). On accumule donc les octets bruts
        # dans self.buffer, et on extrait les trames completes delimitees
        # par des accolades { ... }. C'est le delimiteur reel du protocole.
        self.buffer = ""

        # --- Timer de lecture ---------------------------------------------
        # Toutes les 20 ms, on vide ce qui est arrive sur le port.
        self.timer = self.create_timer(0.02, self.read_serial)

    # ====================================================================
    #  BOUCLE DE LECTURE
    # ====================================================================
    def read_serial(self):
        if self.ser is None:
            return

        try:
            # Lire tout ce qui est dispo (jusqu'a 4096 octets). On ne se fie
            # plus aux retours a la ligne : on lit en brut.
            chunk = self.ser.read(4096)
            if not chunk:
                return

            # Decoder en texte. errors="ignore" : on jette les octets non
            # ASCII isoles plutot que de planter.
            self.buffer += chunk.decode(errors="ignore")

            # Extraire toutes les trames completes { ... } presentes dans le
            # buffer. extract_frames renvoie la liste des trames completes et
            # laisse dans self.buffer le morceau incomplet de la fin (qui sera
            # complete au prochain passage).
            for frame in self.extract_frames():
                self.process_frame(frame)

        except Exception as e:
            self.get_logger().error(f"Serial read error: {e}")

    def extract_frames(self):
        """
        Decoupe self.buffer en trames completes delimitees par { }.
        - Tout ce qui precede la premiere '{' est du bruit -> jete.
        - Chaque couple '{...}' complet est renvoye.
        - Le reste (une '{' ouverte sans '}') est conserve pour la suite.
        """
        frames = []
        while True:
            start = self.buffer.find("{")
            if start == -1:
                # Pas d'accolade ouvrante : tout est du bruit, on vide.
                self.buffer = ""
                break
            end = self.buffer.find("}", start)
            if end == -1:
                # Accolade ouverte mais pas encore fermee : on garde a partir
                # de '{' et on attend la suite au prochain chunk.
                self.buffer = self.buffer[start:]
                break
            # Trame complete trouvee entre start et end inclus.
            frame = self.buffer[start:end + 1]
            frames.append(frame)
            # On continue a chercher apres cette trame.
            self.buffer = self.buffer[end + 1:]
        return frames

    def process_frame(self, frame):
        """Aiguille une trame complete vers le bon decodeur."""
        # On republie toujours la trame brute (tracabilite).
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
            # Trame inconnue : on la signale sans planter (ex: REQ101 barcode).
            self.publish_string(
                self.human_status_pub,
                f"Unknown KL-TCG frame ({command}): {frame}"
            )

    def parse_frame(self, frame):
        """
        Retire les accolades et decoupe sur les virgules.
        Note : une virgule terminale produit un dernier champ vide, c'est
        normal (les decodeurs en tiennent compte via les index).
        """
        clean = frame.strip().strip("{}")
        if not clean:
            return []
        return clean.split(",")

    # ====================================================================
    #  DECODEUR REQ100  (statut periodique du controleur)
    # ====================================================================
    #  [MANUEL] Au demarrage, le controleur envoie {REQ100,...} chaque
    #  seconde. L'appareil externe DOIT repondre {CMD100,...} pour synchro.
    #  Le mapping des champs ci-dessous est repris de la version d'origine
    #  [A CONFIRMER] : il n'a pas ete revalide sur trame REQ100 reelle dans
    #  la derniere capture (peu de REQ100 visibles). A verifier si besoin.
    # ====================================================================
    def handle_req100(self, fields, raw_line):
        # On declenche d'abord la synchro : c'est la raison d'etre de REQ100.
        self.send_cmd100(fields)

        try:
            tool_connected = fields[21]
            tool_enabled = fields[24]
            tool_stop_status = fields[25]
            screw_count = fields[26]
            job = fields[16]
            sequence = fields[17]
            program = fields[19]

            tool_connected_text = "tool connected" if tool_connected == "1" else "tool not connected"
            tool_enabled_text = "tool enabled" if tool_enabled == "1" else "tool disabled"

            decoded = (
                f"KL-TCG status (REQ100): controller alive | "
                f"{tool_connected_text} | {tool_enabled_text} | "
                f"job={job} | sequence={sequence} | program={program} | "
                f"screws={screw_count} | stop_status={tool_stop_status}"
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
    #  FORMAT REEL OBSERVE sur capture (vissage complet) :
    #     {DATA101, temps, col2, valeur_live, }
    #     index:     0      1     2      3      4(vide)
    #
    #  [OBSERVE] index[1] = temps de vissage en secondes (monte 0 -> ~10 s).
    #  [OBSERVE] index[2] = TOUJOURS 0.00 sur toutes les trames capturees.
    #            => champ non utilise. C'est CE champ que l'ancien code lisait
    #               comme "torque", d'ou un couple live toujours nul. CORRIGE.
    #  [OBSERVE] index[3] = valeur live qui varie (montait ~1600, pic ~1668).
    #            => c'est la grandeur temps-reel utile, on publie celle-ci.
    #
    #  [A CONFIRMER] : la nature physique de index[3] n'est PAS certaine.
    #     Ce n'est PAS un couple en N.m (le visseur fait 2.4-25 N.m, pas 1600).
    #     Hypotheses : valeur brute du transducteur (ADC), couple en millemes,
    #     ou RPM. A correler avec une mesure connue avant toute interpretation
    #     physique. Pour l'instant on publie la valeur brute, sans conversion.
    #
    #  [MANUEL] Rappel : DATA101 n'est PAS emis sur WIFI/Ethernet, seulement
    #  en serie. Cohrent avec le fait qu'on le recoit ici en RS232.
    # ====================================================================
    def handle_data101(self, fields, raw_line):
        try:
            # Certaines trames de transition sont tronquees (3 champs au lieu
            # de 5). On exige donc au moins 4 champs avant de lire index[3].
            if len(fields) < 4:
                # Trame courte de debut/fin de cycle : on l'ignore en silence
                # (ce n'est pas une erreur, juste une trame partielle).
                return

            fastening_time = float(fields[1])      # [OBSERVE] temps (s)
            live_value = float(fields[3])          # [OBSERVE] valeur live brute

            # Publication de la valeur live sur le topic Float32.
            torque_msg = Float32()
            torque_msg.data = live_value
            self.live_torque_pub.publish(torque_msg)

            decoded = (
                f"Live data (DATA101): time={fastening_time:.3f}s, "
                f"live_value={live_value} [raw, unit TBD]"
            )
            self.publish_string(self.human_status_pub, decoded)

        except (IndexError, ValueError):
            # float() echoue si le champ est vide/malforme : on signale,
            # on ne plante pas.
            self.publish_string(
                self.human_status_pub,
                f"Could not decode DATA101 frame: {raw_line}"
            )

    # ====================================================================
    #  DECODEUR DATA100  (resultat final apres chaque vissage)
    # ====================================================================
    #  MAPPING REEL OBSERVE (index Python, trame a 29 champs avec vide final) :
    #     [0]  DATA100        type de trame
    #     [1]  annee   [2] mois  [3] jour  [4] heure  [5] min  [6] sec
    #     [11] T01VE00631...   [OBSERVE] Tool ID
    #     [12] C14Z-E02184...  [OBSERVE] Numero de serie
    #     [13] 0000000012      [MANUEL] compteur total de vissages controleur
    #                          (le "column 14" du manuel ; sert a distinguer
    #                           un nouveau resultat d'une repetition).
    #     [16] 01              [OBSERVE] numero de programme
    #     [17] ******          [OBSERVE] nom de programme (non defini ici)
    #     [18] 01              [OBSERVE] outil selectionne
    #     [19] 0000.0000       [OBSERVE] champ a 0 -- PAS le couple.
    #                          => c'est ici que l'ancien code lisait "torque".
    #                             ERREUR. Corrige : on lit [21].
    #     [20] 0               [A CONFIRMER] code unite ? a 0 dans la capture.
    #     [21] 0003.3410       [OBSERVE] valeur resultat non nulle (le "couple"
    #                          ou la grandeur finale mesuree). Champ utile.
    #     [22] 0086.0          [OBSERVE] angle ou temps de serrage final.
    #     [23] 05/05           [OBSERVE] compteur de vis (faites/total).
    #     [25] 1NG-F           [OBSERVE] STATUT du vissage. Contient "NG"
    #                          (echec) ou "OK" (succes). Champ critique.
    #
    #  [A CONFIRMER] : plusieurs champs intermediaires ([14],[15],[24],[26])
    #  ne sont pas nommes avec certitude. Non utilises ici pour eviter
    #  d'inventer leur sens.
    # ====================================================================
    def handle_data100(self, fields, raw_line):
        # Repondre CMD100 : sinon le controleur repete DATA100 en boucle
        # (comportement observe : 74x la meme trame faute d'acquittement).
        self.send_cmd100(fields)

        try:
            total_count = fields[13]      # [MANUEL] compteur total controleur
            program = fields[16]          # [OBSERVE] numero de programme
            program_name = fields[17]     # [OBSERVE] nom de programme
            selected_tool = fields[18]    # [OBSERVE] outil selectionne
            result_value = fields[21]     # [OBSERVE] valeur finale (ex-bug: 19)
            unit_code = fields[20]        # [A CONFIRMER] code unite
            unit = TORQUE_UNITS.get(unit_code, "unknown")
            angle_or_time = fields[22]    # [OBSERVE] angle/temps final
            screw_count = fields[23]      # [OBSERVE] vis faites/total
            status = fields[25]           # [OBSERVE] statut OK / NG

            # Interpretation lisible du statut (sous-chaine, robuste au format
            # exact "1NG-F", "1OK", etc.).
            if "OK" in status:
                status_text = "OK (success)"
            elif "NG" in status:
                status_text = "NG (failure)"
            else:
                status_text = f"unknown ({status})"

            decoded = (
                f"Final result (DATA100): status={status_text} | "
                f"value={result_value} {unit} | "
                f"angle/time={angle_or_time} | screws={screw_count} | "
                f"program={program} | program_name={program_name} | "
                f"tool={selected_tool} | total_count={total_count}"
            )
            self.publish_string(self.final_result_pub, decoded)
            self.publish_string(self.human_status_pub, decoded)

        except IndexError:
            self.publish_string(
                self.human_status_pub,
                f"Incomplete DATA100 frame: {raw_line}"
            )

    # ====================================================================
    #  SYNCHRO CMD100  (la SEULE ecriture serie autorisee)
    # ====================================================================
    #  [MANUEL] Format exact (page 13) :
    #    {CMD100,YEAR,MONTH,DAY,HOUR,MINUTE,SECOND,CHECKSUM,KEYCODE,0,INSTR}
    #    - CHECKSUM = YEAR+MONTH+DAY+HOUR+MINUTE+SECOND  (somme entiere)
    #    - KEYCODE  = CHECKSUM + 5438
    #    - champ 10 = 0 (defaut)
    #    - INSTR    = "Instruction number", identique a celui de la trame
    #                 entrante (REQ100/DATA100). On le recopie depuis le
    #                 dernier champ non vide de la trame recue.
    #
    #  IMPORTANT : ceci n'est PAS une commande de vissage. C'est uniquement
    #  un acquittement + mise a l'heure. Aucune action mecanique declenchee.
    #
    #  CHOIX D'IMPLEMENTATION : on synchronise sur l'heure SYSTEME courante.
    #  [A CONFIRMER] L'usage exact attendu de "instruction number" merite
    #  verification sur trames REQ100 reelles. Par securite, si on ne le
    #  trouve pas, on met "1" (valeur par defaut prudente).
    # ====================================================================
    def send_cmd100(self, fields):
        if not self.enable_sync or self.ser is None:
            return

        try:
            now = datetime.now()
            year = now.year
            month = now.month
            day = now.day
            hour = now.hour
            minute = now.minute
            second = now.second

            checksum = year + month + day + hour + minute + second
            keycode = checksum + KEYCODE_OFFSET

            # Recuperer l'instruction number : dernier champ non vide de la
            # trame entrante. [A CONFIRMER] : a affiner si le manuel precise
            # une position fixe.
            instr = "1"
            for v in reversed(fields):
                if v.strip() != "":
                    instr = v.strip()
                    break

            # Construction de la trame. Format des champs date sur le modele
            # du manuel (annee 4 chiffres, le reste 2 chiffres).
            cmd = (
                f"{{CMD100,{year:04d},{month:02d},{day:02d},"
                f"{hour:02d},{minute:02d},{second:02d},"
                f"{checksum:04d},{keycode:04d},0,{instr}}}"
            )

            # Envoi serie. On termine par \r\n par prudence (le controleur
            # emet ses trames suivies de \r\n ; on imite ce terminateur).
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