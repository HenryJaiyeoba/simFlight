#!/usr/bin/env python3
"""
This script updates all Jupyter notebooks to replace instances of 'oldName' with 'newName'
and 'oldname' with 'newname' in both markdown and code cells.
"""

import json
import os
import re
import sys

def update_notebook(notebook_path):
    """Update references in a Jupyter notebook."""
    with open(notebook_path, 'r', encoding='utf-8') as f:
        notebook = json.load(f)
    
    changes_made = False
    
    # Process each cell in the notebook
    for cell in notebook['cells']:
        if cell['cell_type'] == 'markdown':
            for i, source in enumerate(cell['source']):
                new_source = source.replace('oldName', 'newName').replace('oldname', 'newname')
                if new_source != source:
                    cell['source'][i] = new_source
                    changes_made = True
        
        elif cell['cell_type'] == 'code':
            for i, source in enumerate(cell['source']):
                new_source = source.replace('oldname.', 'newname.').replace('oldName', 'newName')
                if new_source != source:
                    cell['source'][i] = new_source
                    changes_made = True
    
    if changes_made:
        with open(notebook_path, 'w', encoding='utf-8') as f:
            json.dump(notebook, f, indent=1)
        print(f"Updated: {notebook_path}")
    else:
        print(f"No changes needed in: {notebook_path}")

def main():
    """Main function to update all notebooks."""
    notebooks_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'notebooks')
    
    if not os.path.exists(notebooks_dir):
        print(f"Error: Notebooks directory not found at {notebooks_dir}")
        sys.exit(1)
    
    notebooks = [f for f in os.listdir(notebooks_dir) if f.endswith('.ipynb')]
    
    if not notebooks:
        print(f"No notebooks found in {notebooks_dir}")
        sys.exit(1)
    
    print(f"Found {len(notebooks)} notebooks to process")
    
    for notebook in notebooks:
        notebook_path = os.path.join(notebooks_dir, notebook)
        update_notebook(notebook_path)
    
    print("Notebook updating complete!")

if __name__ == "__main__":
    main()