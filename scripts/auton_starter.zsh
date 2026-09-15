# MRover autonomy starter project
export AUTON_STARTER_PATH="${AUTON_STARTER_PATH:-$HOME/auton-starter-project}"

source_auton_starter_overlay() {
    if ! typeset -f source_mrover_overlay > /dev/null; then
        print -P '%F{red}%Bsource_mrover_overlay is not defined. Check the mrover shell setup.%b%f'
        return 1
    fi

    local starter_dir="$PWD"
    source_mrover_overlay
    cd "${starter_dir}" || return 1
}

alias auton_starter="cd \$AUTON_STARTER_PATH && source_auton_starter_overlay"
alias build_starter="\$AUTON_STARTER_PATH/scripts/build.sh && auton_starter"
alias clean_starter="rm -rf \$MROVER_ROS2_WS_PATH/build/*/mrover_autonomy_starter \$MROVER_ROS2_WS_PATH/install/*/mrover_autonomy_starter"
