def parse_error(error_num):
    """
    Put the errors in a list given the result number 
    """
    error_list = []
    if error_num & 0x01:
        # simx_return_novalue_flag
        error_list.append("no value returned")
    if error_num & 0x02:
        # simx_return_timeout_flag
        error_list.append("function timed out")
    if error_num & 0x04:
        # simx_return_illegal_opmode_flag
        error_list.append(
            "the specified operation mode is not supported for the given function"
        )
    if error_num & 0x08:
        # simx_return_remote_error_flag
        error_list.append(
            "the function caused an error on the server side (e.g. an invalid handle was specified)"
        )
    if error_num & 0x16:
        # simx_return_split_progress_flag
        error_list.append(
            "the communication thread is still processing previous split command of the same type"
        )
    if error_num & 0x32:
        # simx_return_local_error_flag
        error_list.append("The function caused an error on the client side")
    if error_num & 0x64:
        # simx_return_initialize_error_flag
        error_list.append("simxStart was not yet called")
    return error_list


def err_print(prefix="", message=[]):
    for i in message:
        print("\033[31m[ERROR]:\033[0m " + prefix + " " + i)