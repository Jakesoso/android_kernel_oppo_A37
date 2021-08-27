#!/bin/sh

in="$1"
out="$2"
syscall_macro() {
    abi="$1"
    nr="$2"
    entry="$3"

    # Entry can be either just a function name or "function/qualifier"
    real_entry="${entry%%/*}"
    qualifier="${entry:${#real_entry}}"		# Strip the function name
    qualifier="${qualifier:1}"			# Strip the slash, if any

    echo "__SYSCALL_${abi}($nr, $real_entry, $qualifier)"
}
emit() {
    abi="$1"
    nr="$2"
    entry="$3"
    compat="$4"

    if [ "$abi" == "64" -a -n "$compat" ]; then
	echo "a compat entry for a 64-bit syscall makes no sense" >&2
	exit 1
    fi

    if [ -z "$compat" ]; then
	if [ -n "$entry" ]; then
	    syscall_macro "$abi" "$nr" "$entry"
	fi
    else
	echo "#ifdef CONFIG_X86_32"
	if [ -n "$entry" ]; then
	    syscall_macro "$abi" "$nr" "$entry"
	fi
	echo "#else"
	syscall_macro "$abi" "$nr" "$compat"
	echo "#endif"
    fi
}


grep '^[0-9]' "$in" | sort -n | (
    while read nr abi name entry compat; do
	abi=`echo "$abi" | tr '[a-z]' '[A-Z]'`
        emit "$abi" "$nr" "$entry" "$compat"
    done
) > "$out"
