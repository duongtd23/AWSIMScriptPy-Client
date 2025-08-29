## AWSIM-Script client

A client for AWSIM-Script, to be used with [AWSIM-Labs simulator](https://github.com/duongtd23/AWSIM-Labs/tree/dev).

### Usage
```bash
python client.py <path-to-input-script-or-folder>
```

Make sure to source the folder where Autoware was installed first.
See `python client -h` for more details.
If a folder is given, each script file inside will be processed one by one.
If using with [AW-RuntimeMonitor](https://github.com/duongtd23/AW-RuntimeMonitor/tree/awsimclient), a trace (text data and video) will be saved for each simulation (script file).
