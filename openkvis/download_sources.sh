echo "username:"
read username
wget -r -nH --cut-dirs=4 -np --directory-prefix=download --user=$username --ask-password https://www.doc.ic.ac.uk/~sleutene/software/okvis/download/
mkdir ethzasl_brisk -p
mkdir okvis -p
tar xf download/ethzasl_brisk.v.0.0.tar -C ethzasl_brisk
tar xf download/okvis.v.0.0.tar -C okvis
wstool merge download/okvis.rosinstall 
wstool update
