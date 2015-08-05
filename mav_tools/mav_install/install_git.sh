
sudo apt-get install git -y

# --- SET UP GIT ---

echo "Do you wish to set up git? [y/n]? [Y,n]"
read install_git
if [[ $install_git == "Y" || $install_git == "y" ]]; then
	# fancy bash coloring for git :)
	sh -c 'echo "export GIT_PS1_SHOWDIRTYSTATE=1" >> ~/.bashrc'
	echo "export PS1='\[\033[03;32m\]\u@\h\[\033[01;34m\]:\w\[\033[02;33m\]\$(__git_ps1)\[\033[01;34m\]\\$\[\033[00m\] '" >> ~/.bashrc

	git config --global credential.helper cache
	git config --global push.default current
	git config --global color.ui true

	git config --list

	echo ""

	echo "*** setting up ssh ***"
	if [ ! -f ~/.ssh/id_rsa.pub ]; then
	    echo "ssh key not found!"
	    echo "generating one for you"
	    echo "$ ssh-keygen"
	    ssh-keygen
	else
	    echo "ssh public key found!"
	fi
	echo 

	echo "$ cat ~/.ssh/id_rsa.pub"
	cat ~/.ssh/id_rsa.pub
fi

