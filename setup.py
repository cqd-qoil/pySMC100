from setuptools import setup, find_packages

setup(
    name="smc100",          # The name of your project/package
    version="0.1",              # The version of your project
    py_modules=["smc100"],        # The main module to include (in this case, "main.py")
    install_requires=find_packages(),        # List any dependencies here, if any
    # You can also add a long description, author, license, etc.
    author="Luis Villegas",
    url="https://github.com/luyves/SMC100",  # URL for the project, if applicable
    python_requires=">=3.6",  # Minimum Python version required
)
