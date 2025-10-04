# Configuration file for the Sphinx documentation builder.
#
# For the full list of built-in configuration values, see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# -- Project information -----------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#project-information

project = 'Open Workout System API'
copyright = '2025, Aaron'
author = 'Aaron'
release = '1.0'
version = '1.0'
# The name of the master document (docname, not filename).
master_doc = 'index'

# -- General configuration ---------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#general-configuration

extensions = [
    'sphinxcontrib.httpdomain',  # For pretty HTTP examples
]

templates_path = ['_templates']
exclude_patterns = []



# -- Options for HTML output -------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#options-for-html-output

html_theme = 'furo'
html_title = 'Open Workout System API'
html_static_path = ['_static']

# Furo theme options
html_theme_options = {
    "source_repository": "https://github.com/openworkoutsystem/digital_weight_device/",
    "source_branch": "main",
    "source_directory": "docs/",
    "sidebar_hide_name": True,
    "navigation_with_keys": True,
    "top_of_page_buttons": ["view", "edit"],
}
