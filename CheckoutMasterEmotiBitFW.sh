f="EmotiBit_FeatherWing_depends.txt"
script_name=$(basename "$0")
echo "**** $script_name ****"
echo "Script to read dependency list from $f and checkout master branch for all repos"
echo "ToDo: integrate passing $f into ExtractRepoNames.sh"
./ExtractDepends.sh

eval "$(ssh-agent -s)"
ssh-add ~/.ssh/id_ed25519
readarray -t repos < $f
echo “${repos[@]}”
for i in ${repos[@]}
do
  echo "-- $i --"
  c="cd ../$i"
  echo $c
  pwd
  eval "$c"
  git checkout master
done
