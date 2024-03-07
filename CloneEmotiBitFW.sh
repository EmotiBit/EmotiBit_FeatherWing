f="EmotiBit_FeatherWing_depends.txt"
script_name=$(basename "$0")
echo "**** $script_name ****"
echo "Script to read dependency list from $f and clone repos into ../"
echo "ToDo: read/parse depends directly from library.properties"

eval "$(ssh-agent -s)"
ssh-add ~/.ssh/id_ed25519
readarray -t repos < $f
echo “${repos[@]}”
cd ..
for i in ${repos[@]}
do
  echo "-- $i --"
  if [ -d "$i" ]; then
    echo "$i already exists, skipping..."
  else
    c="git clone git@github.com:EmotiBit/$i.git"
    echo $c
    pwd
    eval "$c"      
  fi
done
