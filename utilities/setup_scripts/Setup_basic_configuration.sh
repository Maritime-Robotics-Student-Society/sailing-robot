#!/bin/bash
# This script set some basic bash aliases and vim comfiguration to feel at home
# when using the sailrobot
# It only includes widely used configurations

cat <<EOT >> ~/.bashrc

# Some useful aliases
alias ls='ls --color=auto'
alias ll='ls -lh'
alias la='ls -Ah'
alias l='ls -CF'
alias sl='ls'
alias lt='ls -tr'
alias llt='ls -ltrh'
alias vi='vim'
alias mv='mv -i'
alias cp='cp -i'

EOT



cat <<EOT >> ~/.vimrc

syntax enable   " enable syntax processing
set background=dark
set smartcase   " case insensitive smart
set number      " show line numbers
set showcmd     " show command in bottom bar
set cursorline  " highlight current line
set expandtab   " tabs are spaces
set lazyredraw  " redraw only when we need to.
EOT



