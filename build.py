#!/usr/bin/env python3

from src.project_config import ProjectConfig
from src.container import Container

def main():
    config = ProjectConfig.build_environment()
    Container(config).build_workspace()

if __name__ == '__main__':
    main()
