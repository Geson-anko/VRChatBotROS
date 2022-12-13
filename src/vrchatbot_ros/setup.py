from setuptools import setup

package_name = "vrchatbot_ros"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Geson Anko",
    maintainer_email="59220704+Geson-anko@users.noreply.github.com",
    description="VRChat Bot System using ROS2",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "speech2text = vrchatbot_ros.speech2text:main",
            "text_chat = vrchatbot_ros.text_chat:main",
            "text2speech = vrchatbot_ros.text2speech:main",
        ],
    },
)
