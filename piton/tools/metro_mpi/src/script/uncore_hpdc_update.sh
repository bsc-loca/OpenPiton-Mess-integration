#!/bin/bash
BRANCH_NAME="ft/fix_SM_MLAC"
URL="git@gitlab-internal.bsc.es:hwdesign/rtl/uncore/openpiton.git"
REPO_NAME="uncore_repo"

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
REPO_DIR="$SCRIPT_DIR/$REPO_NAME"
TILE_DIR="$REPO_DIR/piton/design/chip/tile"
ARIAN_DIR="$TILE_DIR/ariane"
SARG_DIR="$TILE_DIR/sargantana"


git clone $URL $REPO_NAME
#checkout default branch
cd $REPO_DIR
git pull
git submodule update --init --recursive
git submodule update --init --recursive

#create new branch in OP
git branch $BRANCH_NAME
git checkout $BRANCH_NAME

#create new branch in ariane
cd $ARIAN_DIR
git fetch
git branch $BRANCH_NAME
git checkout $BRANCH_NAME

#add hpdc
cd cv-hpdcache
git fetch
git checkout $BRANCH_NAME

cd ..
git add cv-hpdcache 
git commit -m "update hpdcache with $REPO_NAME"


#create new branch in sargantana
cd $sarg_DIR
git fetch
git branch $BRANCH_NAME
git checkout $BRANCH_NAME

#add hpdc
cd rtl/dcache
git fetch
git checkout $BRANCH_NAME

cd ..
git add dcache 
git commit -m "update hpdcache with $REPO_NAME"


cd $TILE_DIR
add ariane
add sargantana

git commit -m "update hpdcache with $REPO_NAME"




  
