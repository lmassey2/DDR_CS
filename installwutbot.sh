#!/bin/bash

#Copyright <2019> <LIAM R. MASSEY>
#
#Redistribution and use in source and binary forms, with or without modification, are permitted 
#provided that the following conditions are met:
#
#1. Redistributions of source code must retain the above copyright notice, this list of conditions 
#and the following disclaimer.
#
#2. Redistributions in binary form must reproduce the above copyright notice, this list of 
#conditions and the following disclaimer in the documentation and/or other materials provided with 
#the distribution.
#
#3. Neither the name of the copyright holder nor the names of its contributors may be used to 
#endorse or promote products derived from this software without specific prior written permission.
#
#THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR 
#IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND 
#FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR 
#CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL 
#DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, 
#DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER 
#IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT 
#OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

INSTALL_DIR=/etc/wutbot
INSTALL_DIR_BIN=/usr/bin
BIN_FILE=wutbotpi
PY_FILE=pipython.py


DoInstall()
{
	echo 'Installing source code in:'
	echo $INSTALL_DIR
	echo 'Installing bin in:'
	echo $INSTALL_DIR_BIN
	sudo mkdir -p $INSTALL_DIR
	sudo cp $PY_FILE $INSTALL_DIR
	sudo cp $BIN_FILE $INSTALL_DIR_BIN
}
DoUninstall()
{
	echo 'Uninstalling W.U.T. Bot'
	sudo rm -rf $INSTALL_DIR
	sudo rm $INSTALL_DIR_BIN/$BIN_FILE
}
ShowUsage()
{
	echo '------------------------------------------------------------'
	echo '--This is for installing and uninstalling the Raspberry Pi--'
	echo '--controller for the W.U.T. Bot.  Accepted commands are:  --'
	echo '------------------------------------------------------------'
	echo 'install - installs the controller script'
	echo 'uninstall - removes the script and source code directories'
}


case "$1" in
	'install' )
		DoInstall
	;;
	'uninstall' )
		DoUninstall
	;;
	* )
		ShowUsage
	;;
esac

exit 0