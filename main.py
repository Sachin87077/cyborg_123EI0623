import cv2 as cv

def shop_detail(img) -> list:



    new = []
    for i in range(0, 600, 100):
        shop = img[110:190, i + 110:i + 190]
        gray = cv.cvtColor(shop, cv.COLOR_BGR2GRAY)
        threshold = cv.adaptiveThreshold(gray, 255, cv.ADAPTIVE_THRESH_GAUSSIAN_C, cv.THRESH_BINARY_INV, 11, 5)

        contours, _ = cv.findContours(threshold, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_NONE)
        for contour in contours:
            approx = cv.approxPolyDP(contour, 0.01 * cv.arcLength(contour, True), True)

            if len(approx) == 3:
                shape = "Triangle"
                ax = 110
                ay = 111
            elif len(approx) == 4:
                shape = "Square"
                ax = 110
                ay = 110
            else:
                ax = 110
                ay = 110
                shape = "Circle"

            shopNum = i // 100 + 1
            M = cv2.moments(contour)
            x = int(M['m10'] / M['m00']) + ax + i
            y = int(M['m01'] / M['m00']) + ay
            sky = [255,255,0]
            orange=[0,127,255]
            
            pink = [180,0,255]
            green = [0,255,0]

            color = list(img[y, x])

            if color == sky:
                col = "Skyblue"
            elif color == pink:
                col = "Pink"
            elif color == orange:
                col = "Orange"
            elif color == green:
                col = "Green"
            else:
                col = "tmkc"

            lst = [f'Shop_{str(shopNum)}', col, shape, [x, y]]
            new.append(lst)
    new.sort()
    return new


def signals(img) -> list:

    map = {"1": "A", "2": "B", "3": "C", "4": "D", "5": "E", "6": "F", "7": "G"}
    new = []
    for i in range(100, 800, 100):
        for j in range(100, 800, 100):
            if list(img[j, i]) == [0, 0, 255]:
                mapp = (map[[elt for elt in str(i)][0]] + [elt for elt in str(j)][0])
                new.append(mapp)
    return new


def start(img) -> list:
    mp = {"1": "A", "2": "B", "3": "C", "4": "D", "5": "E", "6": "F", "7": "G"}
    new = []
    for i in range(100, 800, 100):
        for j in range(100, 800, 100):
            if list(img[j, i]) == [0, 255, 0]:
                mapp = (mp[[elt for elt in str(i)][0]] + [elt for elt in str(j)][0])
                new.append(mapp)
    return new


def vertical_const(img):
    mp = {"1": "A", "2": "B", "3": "C", "4": "D", "5": "E", "6": "F", "7": "G"}
    new = list()
    for i in range(100, 800, 100):
        for j in range(150, 750, 100):
            if list(img[j, i]) != [0, 0, 0]:
                x = mp[[elt for elt in str(i)][0]]
                y1 = [elt for elt in str(j - 50)][0]
                y2 = [elt for elt in str(j + 50)][0]
                new.append(f'{x}{y1}-{x}{y2}')

    return new


def horizontal_const(img):
    mp = {"1": "A", "2": "B", "3": "C", "4": "D", "5": "E", "6": "F", "7": "G"}
    new = list()
    for i in range(150, 750, 100):
        for j in range(100, 800, 100):
            if list(img[j, i]) != [0, 0, 0]:
                x1 = mp[[elt for elt in str(i - 50)][0]]
                x2 = mp[[elt for elt in str(i + 50)][0]]
                y = [elt for elt in str(j)][0]
                new.append(f'{x1}{y}-{x2}{y}')

    return new


def detect_arena_parameters(maze_image):
    
    arena_parameters = {}
    arena_parameters['traffic_signals'] = signals(maze_image)
    arena_parameters['start_node'] = start(maze_image)
    arena_parameters['horizontal_roads_under_construction'] = horizontal_const(maze_image)
    arena_parameters['vertical_roads_under_construction'] = vertical_const(maze_image)
    shops = []
    shops = shop_detail(maze_image)
    sorted_data = sorted(shops, key=lambda x: (int(x[0].split('_')[1]), x[1]))
    arena_parameters['medicine_packages_present'] = sorted_data

    return arena_parameters

