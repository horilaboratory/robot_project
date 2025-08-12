# Configuration file for the Sphinx documentation builder.
#
# For the full list of built-in configuration values, see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# -- Project information -----------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#project-information

project = 'robot_project'
copyright = '2025, Gai Nakatogawa'
author = 'Gai Nakatogawa'
release = '0.1.0'

# -- General configuration ---------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#general-configuration

extensions = [
    'pydata_sphinx_theme',
    'sphinx_tabs.tabs',
    'sphinx_copybutton',
    'sphinx_design',
    'sphinx.ext.autosectionlabel',
    'sphinx_togglebutton',
    'sphinx_comments',
    'myst_parser'
]

templates_path = ['_templates']
exclude_patterns = []

language = 'ja'

# -- Options for HTML output -------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#options-for-html-output

#html_theme = 'sphinx_rtd_theme'
html_theme = 'pydata_sphinx_theme'
html_static_path = ['_static']
html_logo = "_static/logo.png"

# source doc format
source_suffix = {
    '.rst': 'restructuredtext',
    '.md': 'markdown',
}
# comment func
comments_config = {
   "hypothesis": True
}

copybutton_prompt_text = r"$ |>>> |\.\.\. "
copybutton_remove_prompts = True
