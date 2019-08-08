cursor=0
tput cup 0 0
clear

while true
do
	for t1 in "TestTerm2-AI" "TestTerm4-AI" "TestTerm5-BI"
	do
		for var in 1 2 3 4
		do
			echo -e "\r$(caget ${t1}:${var}.RVAL)               "
			cursor=$((cursor+=1))
			tput cup ${cursor} 0
		done
	done
	cursor=0
done