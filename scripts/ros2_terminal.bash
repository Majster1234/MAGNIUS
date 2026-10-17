# Plik startowy interaktywnego Bash; używany przez start_ros2.sh przez --rcfile.
# Nie uruchamiaj go jako osobnego skryptu.

# Zachowaj ustawienia użytkownika, a następnie wczytaj środowisko projektu.
if [[ -r "$HOME/.bashrc" ]]; then
    source "$HOME/.bashrc"
fi

if [[ -z "${ROBOT_ROS2_WORKSPACE:-}" || -z "${ROBOT_ROS2_SETUP:-}" ]]; then
    printf 'Błąd: uruchom tę sesję przez start_ros2.sh.\n' >&2
    return 1
fi
if ! source "$ROBOT_ROS2_SETUP"; then
    printf 'Błąd: nie udało się wczytać ROS 2 Humble.\n' >&2
    return 1
fi

# local_setup dodaje tylko bieżący workspace po jawnie wczytanym Humble.
if [[ -r "$ROBOT_ROS2_WORKSPACE/install/local_setup.bash" ]]; then
    if ! source "$ROBOT_ROS2_WORKSPACE/install/local_setup.bash"; then
        printf 'Błąd: nie udało się wczytać lokalnego workspace.\n' >&2
        return 1
    fi
else
    printf 'Workspace wymaga zbudowania: colcon build --symlink-install\n'
fi

if ! cd -- "$ROBOT_ROS2_WORKSPACE"; then
    printf 'Błąd: katalog workspace jest niedostępny.\n' >&2
    return 1
fi

printf '\n%s\nROS_DISTRO=%s\nWorkspace: %s\n\n' \
    "${ROBOT_ROS2_TITLE:-ROS 2}" "${ROS_DISTRO:-nieustalone}" "$PWD"

# Rola pozostaje widoczna także wtedy, gdy ustawienia użytkownika zmieniają tytuł okna.
PS1="[ROS2:${ROBOT_ROS2_ROLE:-session}] ${PS1-\w\$ }"

case "${ROBOT_ROS2_ROLE:-}" in
    application)
        if (( $# > 0 )); then
            printf 'Uruchamiam: '
            printf '%q ' "$@"
            printf '\n'
            # Argumenty są przekazywane bez eval i bez składania polecenia w tekst.
            if "$@"; then
                printf '\nPolecenie zakończone (kod 0). Terminal pozostaje otwarty.\n'
            else
                robot_ros2_status=$?
                printf '\nPolecenie zakończone (kod %s). Terminal pozostaje otwarty.\n' \
                    "$robot_ros2_status"
                unset robot_ros2_status
            fi
        else
            printf 'Terminal aplikacji jest gotowy. Tu uruchomisz przyszły ros2 launch.\n'
        fi
        ;;
    diagnostics)
        printf '%s\n' \
            'Przydatne polecenia diagnostyczne:' \
            '  ros2 node list' \
            '  ros2 topic list' \
            '  ros2 topic echo /joint_states' \
            '  ros2 control list_controllers  (po uruchomieniu controller_manager)'
        ;;
    development)
        printf '%s\n' \
            'Budowanie workspace:' \
            '  colcon build --symlink-install' \
            '  source install/local_setup.bash' \
            'Po przebudowaniu wczytaj local_setup.bash również w pozostałych terminalach.'
        ;;
esac
