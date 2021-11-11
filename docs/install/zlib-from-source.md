Potential instructions for installing zlib from source.
``` bash
mkdir tmp
cd tmp
wget https://zlib.net/zlib-1.2.11.tar.gz
tar -xzf zlib-1.2.11.tar.gz 
cd zlib-1.2.11/
./configure --prefix=/usr/local/
make
sudo make install
```