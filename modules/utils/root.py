import datetime
import os
from pathlib import Path

class _RootManager:
    _instance = None

    def __new__(cls, *args, **kwargs):
        if not cls._instance:
            cls._instance = super().__new__(cls)
            cls.project_root = None
            cls.workspace_root = None
            cls.data_root = None
        return cls._instance

    def update_root(self, workspace_root: str = None) -> None:
        if workspace_root is None:
            self.project_root = self.get_project_root()
            current_datetime = datetime.datetime.now()
            formatted_date = current_datetime.strftime("%Y-%m-%d_%H-%M-%S")
            self.workspace_root = self.project_root / f"workspace/{formatted_date}"
        else:
            self.workspace_root = Path(workspace_root)
        self.data_root = self.workspace_root / "data"

    def get_project_root(self):
        """Search upwards to find the project root directory."""
        current_path = Path.cwd()
        while True:
            if ((current_path / ".git").exists()
                or (current_path / ".project_root").exists()
                or (current_path / ".gitignore").exists()
            ):
                # use metagpt with git clone will land here
                return current_path
            parent_path = current_path.parent
            if parent_path == current_path:
                # use metagpt with pip install will land here
                cwd = Path.cwd()
                return cwd
            current_path = parent_path

root_manager = _RootManager()
