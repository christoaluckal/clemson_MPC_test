
string="o"
for (( track=0; track<${#string}; track++)); do
    curr_track=${string:$track:1}
    echo $curr_track
    if [[ $curr_track == "i" ]]; then
        for i in 1 2 3 4 5
        do
            for j in 8 10 12
            do
                python3 feasible_mpc.py i 6 1 0.6 0.6 0.8 0.1 0.001 0.001 1 1 $i $j
            done
        done
    fi

    if [[ "$curr_track" == "s" ]]; then
        for i in 1 2 3 4 5
        do
            for j in 8 10 12
            do
                python3 feasible_mpc.py s 4 1.5 2 2 0.2 0.3 1 1 1 1 $i $j
            done
        done
    fi

    if [[ "$curr_track" == "m" ]]; then
        for i in 1 2 3 4 5
        do
            for j in 8 10 12
            do
                python3 feasible_mpc.py m 5 1 10 10 10 0 0.5 0.5 0.01 0.01 $i $j
            done
        done
    fi

    if [[ "$curr_track" == "o" ]]; then
        for i in 1 2 3 4 5
        do
            for j in 8 10 12
            do
                python3 feasible_mpc.py o 4 1.5 2 2 2 0.01 0.1 0.1 0 0 $i $j
            done
        done
    fi
done
