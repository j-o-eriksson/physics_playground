from distutils.core import setup

setup(
    name="blackhole",
    version="0.1",
    description="TBA",
    author="Jon Doe",
    packages=["phyplay"],
    install_requires=[
        "numpy",
        "pyquaternion",
        "wheel",
        "pysdl2",
        "pysdl2-dll",
    ],
)
