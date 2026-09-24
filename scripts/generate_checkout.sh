#!/bin/bash

# This script generates clone_or_pull commands for repositories in src

THIS_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
SRC_DIR="$THIS_DIR/../src"

for dir in "$SRC_DIR"/*; do
    if [ -d "$dir" ] && [ -d "$dir/.git" ]; then
        dirname=$(basename "$dir")
        url=$(git -C "$dir" remote get-url origin)
        branch=$(git -C "$dir" rev-parse --abbrev-ref HEAD)
        
        # fallback to main if detached or error
        if [ "$branch" == "HEAD" ] || [ -z "$branch" ]; then
             branch="main" 
        fi
        
        echo "clone_or_pull $branch $url $dirname"
    fi
done
