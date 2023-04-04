#!/usr/bin/env bash
whiptail --title "ROS packages list" --msgbox  "$(rospack list)" 30 70 --scrolltext




PACKAGE=$(whiptail --inputbox "Which package you are using?" 8 39 --title "Package for launch script" 3>&1 1>&2 2>&3)


echo "$PACKAGE"
cd /home/kostya/catkin_ws/src/$PACKAGE/scripts 

# Store the output of lsd in a variable
scripts=$(lsd)

# Use a while loop to read the script names from lsd one line at a time
while IFS= read -r script_name; do
  scripts+=( "$script_name" )
done <<< "$scripts"

# Display the list of scripts in a radio checklist and allow the user to select one
zenity --list --title="Select a script to run" \
      --height=300 \
      --column="Script Name" \
      --separator='\n' "${scripts[@]}"

# If the user selected a script, run it
if [ -n "${scripts[*]}" ]; then
  echo "Running script scripts[@]"
  rosrun $PACKAGE $scripts[@]
else
  echo "The user didn't select a script to run"
fi





