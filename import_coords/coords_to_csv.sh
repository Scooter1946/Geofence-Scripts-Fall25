#!/bin/bash

# EPICS EVEI
# This is a Linux script that reads the data in a coordinates
# log file, and converts it to a CSV format.
# The output is written to a file with the same name as the given
# file, but with a '.csv' file extension.

# To run the script, first run 'chmod +x coords_to_csv.sh' and then
# run './coords_to_csv.sh <logfile>', replacing <logfile> with the
# name of the actual log file.
#
# ---------------------------
# IMPORTANT: READ BELOW!!!
# ---------------------------
# If you are running this script on a logfile that used IMU
# (which should be all of your log files), run the script with the '--imu'
# flag. The command would then look like the following:
#
# ./coords_to_csv.sh --imu <logfile>
#
# For questions, commments or concerns, reach out to Aadhavan Srinivasan (srini193@purdue.edu).

# Exit the program if any command fails. Useful for debugging purposes.
set -xeo pipefail


# This function checks if the program was run with the '--imu' flag.
has_imu_param() {
	imu_flag="--imu"
	cmd_args="$1"

	if grep -q -- "$imu_flag" <<< "$*"; then
		return 0
	else
		return 1
	fi
}

if [ "$1" == "" ]; then
	echo "ERROR: File name must be specified."
	exit
fi

#Last argument to script
file_to_read="${@: -1}"

file_to_write="${file_to_read%.txt}.csv"
temp_file="tmpfile.txt"

echo "LATITUDE,LONGITUDE" > "$file_to_write"

# If the program was run with the --imu flag, then we are looking at a file that contains IMU data in it.
# In this case, here are the additional steps that must be performed before:
# 1. Filter the contents of the file to only get the following things:
#	a. GPS point before update
#	b. Update
#	c. GPS point after update
# 2. Remove the headers for each subsection, and separate each section by a newline.

if has_imu_param "$*"; then
sed '/AVG/,+1 d' "$file_to_read" | sed '/CHANGES/,+1 d' | awk '!/Refresh/ && (/IMU/ || /Lat/)' | sed -e '/BEFORE/c\\' -e '/IMU/d' | tail -n +2 | awk '{if ($1 == "")print ""; else print $2 "," $4}' | awk '!NF{$0="<NEW>"}1' | cat <(echo "<NEW>") - > "$temp_file"
else

# 1. Find the relevant lines of the file (Lat/Long and Refresh rate)
# 2. Remove all newlines in the file (Makes it easier to replace text
# 3. Add a newline after printing a set of Lat/Long/Refresh rate (i.e. before the next 'Latitude' print)
# 4. Append a newline to the end of the file
# 5. Remove the first line of the file, which is just an empty line
# 6. Convert the file to a CSV file by inserting commas between the relevant values
# 7. Output to the correct file

	grep -E 'Lat|GPS' "$file_to_read" | tr '\r\n' ' ' | sed 's/Lat/\nLat/g' | cat - <(echo "") | tail -n +2 | awk '{print $2 "," $4 "," $8}' - >> "$temp_file"
fi

# Remove duplicating entries from file, based on first two columns (only if --imu flag was NOT used)
if ! has_imu_param "$*"; then
	awk -F, '!a[$1 $2]++ {print ;}' "$temp_file"  > "$file_to_write"
else
	cp "$temp_file" "$file_to_write"
fi

# Remove temp file
rm "$temp_file"
