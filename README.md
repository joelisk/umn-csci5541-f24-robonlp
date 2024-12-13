# csci5541-project-template

This is a template webpage for CSCI 5541 NLP class.

# Group name: RoboNLP

# To run the project
- Require Ubuntu 20.04
- Require ROS noetic
- Make sure you have Gazobo ready to go
- Download the ros_kertex workspace from [here](https://docs.github.com/en/pages/getting-started-with-github-pages/creating-a-github-pages-site#creating-your-site), and follow the instruction on that github page.
- Once you finished the all about step, put the "add_object_test.py" and "LLMs_usage.py" into the workspace you created run the following command in two terimal window

In the first window run:
> roslaunch kortex_gazebo spawn_kortex_robot.launch arm:=gen3  gripper:=robotiq_2f_85

In the second window, run:
> rosrun [your workspace name] add_object_test.py


And then run:
> rosrun [your workspace name] LLMs_usage.py  __ns:=my_gen3







# Website deployment: Github pages
## see [here](https://docs.github.com/en/pages/getting-started-with-github-pages/creating-a-github-pages-site#creating-your-site)
- once we have content we can go through the steps to deploy

## site [here](https://joelisk.github.io/umn-csci5541-f24-robonlp/)





#### Feedback from TA:
- Ignore vision (for now)
- Novelty can be found in creating a grid of different LLMs, different model sizes and coming up with some kind of grade for the 
intermediate steps in task sequencing
- Some of the metrics we can consider are # of intermediate steps until success, upper limit of number of intermediate steps before failure