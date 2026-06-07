import importlib
import os

# import all task modules
for module in os.listdir(os.path.dirname(__file__)):
    if module != "__init__.py" and module != "__pycache__":
        importlib.import_module(f"BDXR.tasks.{module.replace('.py', '')}")
