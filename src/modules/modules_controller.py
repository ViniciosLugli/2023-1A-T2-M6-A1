from typing import Any

class ModulesController:
    __modules = []

    def __init__(self):
        pass

    def add(self, module: Any) -> None:
        self.__modules.append(module)

    def access(self, module_name: str) -> Any:
        for module in self.__modules:
            if module.name == module_name:
                return module
        return RuntimeError(f"Module {module_name} not found")