Some explanations and notes regarding the files inside this `main/` folder.

### `ctrl_task.h` and `ctrl_task.c`
The ISR only notifies and the task takes the notification count. A count `1` is normal and `n>1 ` means that the task was late and some interrupts were missed. Stats live in a shared struct that `log_task` reads. There should be no logging in the control loop.