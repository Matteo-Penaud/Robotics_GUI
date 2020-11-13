from math import *
from tkinter import constants
from termcolor import colored
from tkinter import *
from constants import *

### Tkinter Widgets
mainWindow = Tk()
mainWindow.withdraw()
# Creating Widgets
inputPlaceholder = Label(mainWindow, text="Entries (direct mode):")
label1 = Label(mainWindow, text="Input theta1 (°) : ")
label2 = Label(mainWindow, text="Input theta2 (°) : ")
label3 = Label(mainWindow, text="Input theta3 (°) : ")
entry1 = Entry(mainWindow)
entry2 = Entry(mainWindow)
entry3 = Entry(mainWindow)
resultPlaceholder = Label(mainWindow, text="Results :")
resultLabel1 = Label(mainWindow, text="Result x : ")
resultLabel2 = Label(mainWindow, text="Result y : ")
resultLabel3 = Label(mainWindow, text="Result z : ")
result1 = Label(mainWindow, text="0")
result2 = Label(mainWindow, text="0")
result3 = Label(mainWindow, text="0")
updateResultsButton = Button(mainWindow, text="Update Results", command=lambda: updateResults())
updateModeButton = Button(mainWindow, text="Change Mode", command=lambda: updateMode())

currentMode = 'direct'
graphicMode = False


###
  # @brief : This function is used to proceed the Al-Kashi theorem.
  #
  # @param a : value of the first side of the triangle.
  #
  # @param b : value of the second side of the triangle.
  #
  # @param c : value of the third side of the triangle.
  #
  # @return : returns the solution of the applied Al-Kashi theorem.
  #           This value is the value in radians (rad) of the opposite angle of side a.
###
def al_kashi(a, b, c):
    result = ((b ** 2) + (c ** 2) - (a ** 2)) / (2 * b * c)
    return result


###
  # @brief : This function is used to get the final position of the robotic arm according to the motors' angles.
  #
  # @param theta1 : first motor angle.
  #     @note : in radians (rad) if testRealLeg=False, in degrees (°) otherwise.
  #
  # @param theta2 : second motor angle.
  #     @note : in radians (rad) if testRealLeg=False, in degrees (°) otherwise.
  #
  # @param theta3 : third motor angle.
  #     @note : in radians (rad) if testRealLeg=False, in degrees (°) otherwise.
  #
  # @param l1 : length of the first part of the arm.
  #     @note : in millimeters (mm), the value is constL1 by default.
  #
  # @param l2 : length of the second part of the arm.
  #     @note : in millimeters (mm), the value is constL2 by default.
  #
  # @param l3 : length of the third part of the arm.
  #     @note : in millimeters (mm), the value is constL3 by default.
  #
  # @param testRealLeg : tells computeDK whether we use the real leg pattern or not (see origin.pdf).
  #     @note : False by default.
  #
  # @param expectation : expectations for the final coordinates in millimeters (mm). Used only in real leg mode.
  #
  # @return : retruns values of x, y and z in millimeters (mm), inside an array
###
def computeDK(theta1, theta2, theta3, l1=constL1, l2=constL2, l3=constL3, use_rads=False, testRealLeg=False, expectation=[0, 0, 0]):
    #Correcting theta2 and theta3

    # x, y, z Init (safety feature)
    x = 0
    y = 0
    z = 0

    if testRealLeg == False:
        x = (cos(theta1) * (l1 + (l2 * cos(theta2) + l3 * cos(theta3 + theta2))))
        y = sin(theta1) * (l1 + (l2 * cos(theta2) + l3 * cos(theta3 + theta2)))
        z = -(l2 * sin(theta2) + l3 * sin(theta3 + theta2))

    elif testRealLeg == True:
        print("\n Trying : theta1 = {} | theta2 = {} | theta3 = {}".format(theta1, theta2, theta3))
        print(" Should have (+- 1mm) : x = {} | y = {} | z = {}".format(expectation[0], expectation[1], expectation[2]))

        theta2 += theta2Correction
        theta3 += theta2Correction + theta3Correction

        x = cos(radians(theta1)) * (l1 + (l2 * cos(radians(-theta2)) + l3 * sin(radians(theta3 - theta2))))
        y = sin(radians(theta1)) * (l1 + (l2 * cos(radians(-theta2)) + l3 * sin(radians(theta3 - theta2))))
        z = l2 * -sin(radians(theta2)) - l3 * cos(radians(theta3 - theta2))

        print(" Results : x = {:.5} | y = {:.5} | z = {:.5}".format(x, y, z))
        result, color = checkDK([x, y, z], expectation)
        print(colored("{}".format(result), "{}".format(color)))
    
    elif use_rads == True:
        theta2 += theta2Correction
        theta3 += theta2Correction + theta3Correction

        x = cos(radians(theta1)) * (l1 + (l2 * cos(radians(-theta2)) + l3 * sin(radians(theta3 - theta2))))
        y = sin(radians(theta1)) * (l1 + (l2 * cos(radians(-theta2)) + l3 * sin(radians(theta3 - theta2))))
        z = l2 * -sin(radians(theta2)) - l3 * cos(radians(theta3 - theta2))


    return [x, y, z]


###
  # @brief : This function is used to get the angles of the robotic arm according to the ending coordinates.
  #
  # @param x : x pos to reach.
  #
  # @param y : y pos to reach.
  #
  # @param z : z pos to reach.
  #
  # @param l1 : length of the first part of the arm.
  #     @note : in millimeters (mm), the value is constL1 by default.
  #
  # @param l2 : length of the second part of the arm.
  #     @note : in millimeters (mm), the value is constL2 by default.
  #
  # @param l3 : length of the third part of the arm.
  #     @note : in millimeters (mm), the value is constL3 by default.
  #
  # @param testRealLeg : tells computeIK whether we use the real leg pattern or not (see origin.pdf).
  #     @note : False by default.
  #
  # @param expectation : expectations for the angles values, in degrees (°). Used only in real leg mode.
  #
  # @return : retruns values of theta1, theta2 and theta3 in radians (rad), inside an array
###
def computeIK(x, y, z, l1=constL1, l2=constL2, l3=constL3, testRealLeg=False, expectation=[0, 0, 0]):
    ### See leg_proj.pdf to understand all the values ###
    
    #print("trying : x={} y={} z={}".format(x, y, z))


    # NOT USED x1 = cos(theta1) * l1 + bx

    # Calcul of d13 :
    d13 = sqrt((y ** 2) + (x ** 2)) - l1

    # Calcul of d :
    d = sqrt((d13 ** 2) + (z ** 2))

    # Calcul of a :
    a = 0
    if d13 > 0:
        a = atan(z/d13)
    elif d13 < 0:
        a = -atan(z/d13)
    elif d13 == 0:
        a = pi/2

    # Calcul of b :
    b = 0
    if 0 < al_kashi(l3, l2, d) < 1:
        b = acos(al_kashi(l3, l2, d))
    elif (2 * l2 * d) == 0 or (l2 ** 2) + (d ** 2) - (l3 ** 2) == 0:
        b = 0
    
    # Calcul of theta1
    theta1 = atan2(y, x)

    # Calcul of theta2
    theta2 = - a - b

    # Calcul of theta3 :
    if (-1 < al_kashi(d, l2, l3) < 1):
        theta3 = pi - acos(al_kashi(d, l2, l3))
    #elif -1 < (al_kashi(d, l2, l3)) < 1 and x < x1:
       # theta3 = pi + acos(al_kashi(d, l2, l3))
    else:
        theta3 = 0


    ### Entering test mode
    if testRealLeg == True:
        print("\n Trying : x = {} | y = {} | z = {}".format(x, y, z))
        print(" Should have : theta1 = {} | theta2 = {} | theta3 = {}".format(expectation[0], expectation[1], expectation[2]))

        theta2 -= radians(theta2Correction)
        theta3 -= radians(90 - theta2Correction - theta3Correction)
        theta3 = -theta3

        print(" Result : theta1 = {:.5} | theta2 = {:.5} | theta3 = {:.5}".format(degrees(theta1), degrees(theta2), degrees(theta3)))
        results, color = checkIK([degrees(theta1), degrees(theta2), degrees(theta3)], expectation)

        print(colored("{}".format(results), "{}".format(color)))


    return [theta1, theta2, theta3]


### Working
def segment(x1, y1, z1, x2, y2, z2, t, duration, mode='segment'):
    
    # new_t = ((cos(((2*pi)/duration) * t) + 1) * duration) / 2

    # new_t : triangle wave form from 0 to duration
    new_t = fmod(t, duration)

    if new_t > duration/2 and mode == 'segment':
        new_t = duration/2 - (new_t - duration/2)
        new_t *= 2

    x = (x2 - x1) * new_t/duration + x1
    y = (y2 - y1) * new_t/duration + y1
    z = (z2 - z1) * new_t/duration + z1

    results = computeIK(x, y, z)

    return results

### Work in Progress
def triangle(x, z, h, w, t):

    duration = 3
    seg_duration = duration / 3

    y_min = -w/2
    y_max = w/2

    new_t = fmod(t, duration)

    h += z
    
    while 0 <= new_t <= seg_duration:
        return segment(x, y_min, z, x, y_max, z, t, seg_duration, 'triangle')
    while seg_duration <= new_t <= 2*seg_duration:
        return segment(x, y_max, z, x, 0, h, t, seg_duration, 'triangle')
    while 2*seg_duration <= new_t <= duration:
        return segment(x, 0, h, x, y_min, z, t, seg_duration, 'triangle')


### Work in Progress
### Working but not properly (test with 'python sim2.py --mode circle' to figure out)
def circle(x, z, r, t, duration):

    y = r * cos(2*pi*(t/duration))
    z += r * sin(2*pi*(t/duration))

    results = computeIK(x, y, z)

    return results


### 
  # @brief : This function is used to compare the results of computeDK and the expectations.
  #
  # @param results : results returned by computeDK.
  #
  # @param expectations : expected values for computeDK.
  #     @note : format : [x, y, z], in millimeters (same unit as computeDK returns).
  #
  # @return : string telling the user whether the test passed of failed.
###
def checkDK(results, expectation):
    if graphicMode != True:
        if (expectation[0] - 1 <= results[0] <= expectation[0] + 1) and (expectation[1] - 1 <= results[1] <= expectation[1] + 1) and (expectation[2] - 1 <= results[2] <= expectation[2] + 1):
            return " Test passed successfully !", 'green'
        else:
            return " Test failed... Check params", 'red'
    else:
        return " Test passed graphicaly", 'blue'


### 
  # @brief : This function is used to compare the results of computeIK and the expectations.
  #
  # @param results : results returned by computeIK.
  #
  # @param expectations : expected values for computeIK.
  #     @note : format : [theta1, theta2, theta3], in degrees.
  #
  # @return : string telling the user whether the test passed of failed.
###
def checkIK(results, expectation):
    if graphicMode != True:
        if (expectation[0] - 1 <= results[0] <= expectation[0] + 1) and (expectation[1] - 1 <= results[1] <= expectation[1] + 1) and (expectation[2] - 1 <= results[2] <= expectation[2] + 1):
            return " Test passed successfully !", 'green'
        else:
            return " Test failed... Check params", 'red'
    else:
        return " Test passed graphicaly", 'blue'


### Work in Progress
def graphical():
    global graphicMode
    graphicMode = True

    mainWindow.deiconify()
    mainWindow.title("Test kinematics")

    # Constructing Widgets
    inputPlaceholder.grid(row=0, column=0, padx=(5, 5), pady=(5, 0), columnspan=2)
    label1.grid(row=1, column=0, padx=(5, 0), pady=(10 ,5))
    label2.grid(row=2, column=0, padx=(5, 0), pady=5)
    label3.grid(row=3, column=0, padx=(5, 0), pady=(5, 10))
    entry1.grid(row=1, column=1, padx=(0, 5))
    entry2.grid(row=2, column=1, padx=(0, 5))
    entry3.grid(row=3, column=1, padx=(0, 5))

    resultPlaceholder.grid(row=4, column=0, padx=(5, 5), pady=(0, 5), columnspan=2)
    resultLabel1.grid(row=5, column=0, padx=(5, 0), pady=(0, 5))
    resultLabel2.grid(row=6, column=0, padx=(5, 0), pady=(0, 5))
    resultLabel3.grid(row=7, column=0, padx=(5, 0), pady=(0, 5))
    result1.grid(row=5, column=1, padx=(0, 5), pady=(0, 5))
    result2.grid(row=6, column=1, padx=(0, 5), pady=(0, 5))
    result3.grid(row=7, column=1, padx=(0, 5), pady=(0, 5))

    updateResultsButton.grid(row=8, column=0, padx=(5, 5), pady=(0, 5), columnspan=2)
    updateModeButton.grid(row=9, column=0, padx=(5, 5), pady=(0, 5), columnspan=2)

    # Constructing Window
    mainWindow.mainloop()


def updateMode():
    global currentMode
    if currentMode == 'direct':
        currentMode = 'inverse'

        inputPlaceholder.config(text="Entries (inverse mode):")

        label1.config(text="Input x : ")
        label2.config(text="Input y : ")
        label3.config(text="Input z : ")

        resultLabel1.config(text="Result theta1 (°) : ")
        resultLabel2.config(text="Result theta2 (°) : ")
        resultLabel3.config(text="Result theta3 (°) : ")

    elif currentMode == 'inverse':
        currentMode = 'direct'

        inputPlaceholder.config(text="Entries (direct mode):")

        label1.config(text="Input theta1 (°) : ")
        label2.config(text="Input theta2 (°) : ")
        label3.config(text="Input theta3 (°) : ")

        resultLabel1.config(text="Result x : ")
        resultLabel2.config(text="Result y : ")
        resultLabel3.config(text="Result z : ")


def updateResults():
    inputs = [float(entry1.get()), float(entry2.get()), float(entry3.get())]

    if currentMode == 'direct':
        results = computeDK(inputs[0], inputs[1], inputs[2], 51, 63.7, 93, True)
    elif currentMode == 'inverse':
        results = computeIK(inputs[0], inputs[1], inputs[2], 51, 63.7, 93, True)
        results = [degrees(results[0]), degrees(results[1]), degrees(results[2])]
    else:
        results = {'ERROR', 'ERROR', 'ERROR'}

    result1.config(text=results[0])
    result2.config(text=results[1])
    result3.config(text=results[2])


### Main function
def main():
    # Clear the console and reset cursor pos
    print(chr(27) + "[2J")
    print(chr(27) + "[H" + "Using real leg mode... Run sim2.py tu use the simulation.")
    print("All the values used in this mode referes to the real leg values. See origin.pdf")
    print("Units : angles in degrees (°), distances in millimeters (mm).\n")


    print(colored("Testing Direct Kinematics...", attrs=['bold']))

    computeDK(0, 0, 0, 51, 63.7, 93, True, [118.79, 0, -115.14])
    computeDK(90, 0, 0, 51, 63.7, 93, True, [0, 118.79, -115.14])
    computeDK(180, -30.501, -67.819, 51, 63.7, 93, True, [-64.14, 0.0, -67.79])
    computeDK(0, -30.645, 38.501, 51 ,63.7, 93, True, [203.23, 0.0, -14.30])

    print(colored("\nDirect Kinematics Tests Finished...", attrs=['bold']))

    print(colored("\n\nTesting Inverse Kinematics...", attrs=['bold']))

    computeIK(118.79, 0, -115.14, 51, 63.7, 93, True, [0, 0, 0])
    computeIK(0, 118.79, -115.14, 51, 63.7, 93, True, [90, 0, 0])
    computeIK(-64.14, 0.0, -67.79, 51, 63.7, 93, True, [180, -30.501, -67.819])
    computeIK(203.23, 0.0, -14.30, 51, 63.7, 93, True, [0, -30.645, 38.501])

    print(colored("\nInverse Kinematics Tests Finished...\n", attrs=['bold']))

    print("Would you like to run graphical tool now (it will use real leg referencial) ? (Y/n)")

    while 1:
        final = input()
        if final == 'n' or final == 'N':
            print("Leaving ...\n")
            break
        elif final == 'y' or final == 'Y' or final == '':
            print("Starting graphic interface...")
            mainWindow.bind('<Return>', lambda event: updateResults())
            graphical()
            break
        else:
            print("Input not correct, type 'Y'/'y' for Yes, or 'N'/'n' for No")


if __name__ == "__main__":
    main()
