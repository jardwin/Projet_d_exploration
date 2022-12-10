#!/bin/bash
INPUT=/tmp/menu.sh.$$

trap "rm $INPUT; exit" SIGHUP SIGINT SIGTERM

while true
do

dialog --clear \
    --backtitle "SPIDERTRON" \
    --title "Mouvement" \
    --menu "C'est vous qui controller l'araignée." \
    15 51 4 \
    1 "Marche avant" \
    2 "Tourner à gauche" \
    3 "Position de calibrage" \
    4 "Assis" \
    5 "Montage" \
    6 "Quitter" 2> "${INPUT}"

menuitem=$(<"${INPUT}")

case $menuitem in
1)
    ros2 topic pub /spider_move_order std_msgs/msg/Int32 "data: 0" --once
    ;;
2)
    ros2 topic pub /spider_move_order std_msgs/msg/Int32 "data: 1" --once
    ;;
3)
    ros2 topic pub /spider_move_order std_msgs/msg/Int32 "data: 2" --once
    ;;
4)
    ros2 topic pub /spider_move_order std_msgs/msg/Int32 "data: 3" --once
    ;;
5)
    ros2 topic pub /spider_move_order std_msgs/msg/Int32 "data: 4" --once
    ;;
6)
    break
    ;;
esac
done

[ -f $INPUT ] && rm $INPUT
clear