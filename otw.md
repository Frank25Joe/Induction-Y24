Use ssh to login to bandit.labs.overthewire.org on port 2220. -p is used to specify the port and -l to specify the login name (bandit0).

Level 0: The ls command lists all the files in the working directory. cat concatenates 2 files and prints out the content. So cat followed by a single filename just prints the file contents which is the password for level1.

Level 1: For filenames starting with - cat won't work directly. We should use cat ./< filename starting with - >.

Level 2: For filenames with spaces we should enclose the entire filename with "" so that the cat command will read the entire filename. Eg: cat "filename with spaces"

Level 3: The password is in a hidden file (filename starts with .) in inhere directory. Use cd inhere to enter the inhere directory. The ls command does not list hidden files in a directory but the ls -a (list all) does. After getting the name of the hidden file, normally using cat works.

Level 4: To find the only human readable file in the inhere directory, we can use the file command which lists the filetype of specified file. file ./* lists the filetype of all the files in the current directory. A filetype of ascii text is human readable.

Level 5: find -size 1033c finds the location of a file of 1033 bytes (the c specifies bytes).

Level 6: find / -size 33c -user bandit7 -group bandit6 gives the files that satisfy the given conditions. Here the / after find is to search from the root directory in order to search the entire server. But this command prints error messages for all the files we don't have access to. This can be avoided by adding 2>/dev/null to redirect all those error messages to a null file.

Level 7: The grep command prints the line which has a part matching a particular pattern or string. So grep millionth data.txt prints the line containing the word millionth.

Level 8: Using sort data.txt | uniq -u prints the password. Here, the sort command sorts the lines of the text file. This output is then piped as the input of the uniq -u command which prints only the unique lines of sortes text.

Level 9: The strings command prints all the sequences of human readable characters in the file. This output when piped into the grep == command prints only those human readble lines which have a sequence of = in them.

Level 10: The base64 command with option -d decodes and prints the contents of a base64 encoded file.

Level 11: The command cat data.txt | tr 'a-z A-Z' 'n-za-m N-ZA-M' prints the unrotated password. Here, tr 'a-z A-Z' 'n-za-m N-ZA-M' maps the half the letters a, b,..,z to n,...z and the 2nd half to a,b,...m; and same for capital letters.

Level 12: First we make a temporary directory to make working easier. We do this by using mktemp -d and copy data.txt into it. The file data.txt is a hexadump which can be decoded using the command xxd -r data.txt compressed which converts the hexadump into binary and stores it in a new file called compressed. Using file compressed, we see that it is of type gzip compressed. Rename compressed to compressed.gz and decompress using gzip -d compressed.gz. Then file compressed shows that it is bzip2 compressed. Rename to compressed.bz2 and decompress using bzip2 -d compressed.bz2. Then file command shows that it is a POSIX tar archive. Use tar -r -f comp to extract it. ls shows the file data5.bin and it is another archive. Keep repeating the above steps till we get the password.

Level 13: We get the private key using cat sshkey.private and make a new file (say named "a") and copy the key into it. After logging out, we use the identity file option (-i) of ssh to login to bandit14 using the private key using the command ssh -i a bandit.labs.overthewire.org -p 2220 -l bandit14.

Level 14: We can submit the password using netcat. The command nc localhost 30000 prompts the user to enter information which will be sent to the overthewire server at port 30000. The information to be entered is the current level password.

Level 15: Similar to the previous level, use command openssl s_client localhost:30001 followed by the current password. The reason we can't use nc here is because nc can't handle SSL/TLS encryption which this level requires.
