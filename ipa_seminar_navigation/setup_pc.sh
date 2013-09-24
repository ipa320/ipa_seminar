# sudo mkdir /u

fusermount -u /u
sudo chown $USER /u
sshfs raw3-3-pc2:/u /u
