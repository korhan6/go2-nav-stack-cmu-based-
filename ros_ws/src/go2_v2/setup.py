from glob import glob

from setuptools import setup

setup(
    name="go2_v2",
    version="2.0.0",
    packages=["go2_v2"],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/go2_v2"]),
        ("share/go2_v2", ["package.xml"]),
        ("share/go2_v2/launch", glob("launch/*")),
        ("share/go2_v2/config", glob("config/*")),
        ("share/go2_v2/rviz", glob("rviz/*")),
        ("share/go2_v2/urdf", glob("urdf/*")),
        ("share/go2_v2/meshes", glob("meshes/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="korhan",
    maintainer_email="info@velkine.com",
    description="Go2 v2 sobre el stack de CMU",
    license="BSD-3-Clause",
    entry_points={"console_scripts": [
        "sensores = go2_v2.sensores:main",
        "guardian = go2_v2.guardian:main",
        "herramientas = go2_v2.herramientas:main",
        "modelo = go2_v2.modelo:main",
        "teclado = go2_v2.teclado:main",
        "fantasmas = go2_v2.fantasmas:main",
        "mapa = go2_v2.mapa:main",
        "metas = go2_v2.metas:main",
        "ps4 = go2_v2.ps4:main",
        "calibrar_fantasmas = go2_v2.calibrar_fantasmas:main",
    ]},
)
