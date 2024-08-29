f="EmotiBit_FeatherWing_depends.txt"
script_name=$(basename "$0")
echo "**** $script_name ****"
echo "Script to read from library.properties and write repo list to $f"
echo
echo "rm -f ./$f"
rm -f ./$f
echo
eqPos=8
keyword="depends="
echo "Writing repos to file:"
while IFS= read -r line; do
    #echo "-- $line"
    test=${line:0:$eqPos}
    #echo $test
    #echo
    if [ "$test" = $keyword ]; then
      cutPos=$((eqPos))
      rest=$(echo "$line" | cut -c $cutPos-)
      #echo $rest
      IFS=',' read -ra repos <<< "$rest"
      for _repo in "${repos[@]}"; do
        # remove the first char
        repo=$(echo "$_repo" | cut -c 2-)
        echo $(tr ' ' '_' <<<"$repo")
        echo $(tr ' ' '_' <<<"$repo") >> ./$f
      done
    fi    
done < library.properties