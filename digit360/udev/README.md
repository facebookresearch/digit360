# Installing Digit 360 `udev` Rules

Copy rules file to `udev` rules,

```
sudo cp 50-digit360.rules /etc/udev/rules.d/
```

Then reload and retrigger rules,

```
sudo udevadm control --reload && sudo udevadm trigger
```

