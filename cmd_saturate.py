def cmd_saturate(cmd,cmd_max,cmd_min):
	if cmd > cmd_max:
		cmd = cmd_max
	elif cmd < cmd_min:
		cmd = cmd_min
	return cmd
