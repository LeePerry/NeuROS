import os
import sys
sys.path.insert(0, os.path.abspath(os.path.join('..', '..')))

project = 'NeuROS'
copyright = '2024, Lee Perry'
author = 'Lee Perry'
release = '1.0.0'
extensions = ['sphinx.ext.autodoc', 'sphinx.ext.napoleon', 'sphinxarg.ext']
templates_path = ['_templates']
exclude_patterns = []
html_theme = 'alabaster'
html_static_path = []
