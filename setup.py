from pybind11.setup_helpers import Pybind11Extension, build_ext
from setuptools import setup

__version__ = "0.0.1"

ext_modules = [
    Pybind11Extension(
        "phycpp",
        ["phycpp/main.cpp", "phycpp/particle.cpp"],
        include_dirs=["ext/glm"],
        # Example: passing in the version to the compiled code
        define_macros=[("VERSION_INFO", __version__)],
    ),
]

setup(
    name="blackhole",
    version=__version__,
    description="TBA",
    author="Jonatan Eriksson",
    packages=["phyplay", "phyplay.particle"],
    install_requires=[
        "pytest",
        "wheel",
        "numpy",
        "pyquaternion",
        "pysdl2",
        "pysdl2-dll",
    ],
    ext_modules=ext_modules,
    extras_require={"test": "pytest"},
    # Currently, build_ext only provides an optional "highest supported C++
    # level" feature, but in the future it may provide more features.
    cmdclass={"build_ext": build_ext},
    zip_safe=False,
    python_requires=">=3.7",
)
