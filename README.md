## AWSIM-Script client

A client for AWSIM-Script, to be used with [AWSIM-Labs simulator](https://github.com/duongtd23/AWSIM-Labs/tree/dev).

### Usage
We recommend specifying your scenarios using Python. Some example scenarios are available in [scenarios](scenarios) folder.
For instance, to execute a ziczac scenario, use the following command:
```bash
python -m scenarios.random.ziczac2
```
Make sure to source the folder where Autoware was installed first.

To simulate a scenario specified in a `.script` file:
```bash
python script_file_manager.py <path-to-input-script-or-folder>
```

See `python script_file_manager.py -h` for more details.
If a folder is given, each script file inside will be processed one by one.
If using with [AW-RuntimeMonitor](https://github.com/duongtd23/AW-RuntimeMonitor/tree/awsimclient), a trace (text data and video) will be saved for each simulation (script file).
