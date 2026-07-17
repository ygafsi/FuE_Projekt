from setuptools import find_packages, setup


package_name = "screwdriver_driver"


setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(
        exclude=["test"],
    ),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        (
            "share/" + package_name,
            ["package.xml"],
        ),
        (
            "share/" + package_name + "/config",
            ["config/screwdriver_config.yaml"],
        ),
    ],
    install_requires=[
        "setuptools",
    ],
    zip_safe=True,
    maintainer="robot",
    maintainer_email="robot@example.com",
    description="ROS2 driver for the KILEWS KL-TCG controller",
    license="Apache-2.0",
    tests_require=[
        "pytest",
    ],
    entry_points={
        "console_scripts": [
            "kl_tcg_driver = "
            "screwdriver_driver.kl_tcg_driver:main",
            "screwdriver_gui = screwdriver_driver.screwdriver_gui:main",
        ],
    },
)