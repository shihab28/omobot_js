import cv2, os, json
import numpy as np

tfTestFile = "src/mobot_pkg/data/base_footprint_points.txt"


with open(tfTestFile, 'r') as rf:
    fileLines = rf.readlines()


firstIndex = -1
Datas = {
}

sampleNo = 0
positions = []
pixel_per_meter = [97, 98]
map_origin_pixel = [218, 370]
for ind, line in enumerate(fileLines):
    line = line.strip()
    if line.startswith("At time"):
        if firstIndex == -1:
            firstIndex =  ind
        Time = int(line.replace("At time ", "").strip().split(".")[0])
        Translation = [float(vals.strip())for vals in fileLines[ind+1].replace("- Translation: ", "").replace("[", "").replace("]", "").strip().split(",")]
        Rotation = [float(vals.strip()) for vals in fileLines[ind+2].replace("- Rotation: in Quaternion ", "").replace("[", "").replace("]", "").strip().split(",")],

        if firstIndex == ind:
            Datas[sampleNo] = {
                "time": Time,
                "Translation": Translation,
                "Rotation": Rotation
                }
            # print(Datas[sampleNo])
            positions.append([ int(Translation[1]*pixel_per_meter[0]), int(Translation[0]*pixel_per_meter[1]) ])
            sampleNo += 1
        else:
            if Translation != Datas[sampleNo - 1]["Translation"]  or Rotation != Datas[sampleNo - 1]["Rotation"]:
                Datas[sampleNo] = {
                "time": Time,
                "Translation": Translation,
                "Rotation": Rotation 
                }
                # print(Datas[sampleNo])
                positions.append([ int(Translation[1]*pixel_per_meter[0]), int(Translation[0]*pixel_per_meter[1]) ])

                sampleNo += 1

image = cv2.imread("src/mobot_pkg/data/Map_Annotated.png")
color = (0, 255, 0)
for ind, pos in enumerate(positions):
    
    if ind != len(positions) - 1:
        srt = (pos[1]+map_origin_pixel[1] ,map_origin_pixel[0] - pos[0] )
        end_tem = positions[ind+1]
        end = (end_tem[1]+map_origin_pixel[1], map_origin_pixel[0] - end_tem[0])
        print(srt, end)
        
        # if np.linalg.norm(np.array(srt)-np.array(end)) < 50:

        image = cv2.arrowedLine(image, srt, end, color, 1, tipLength=1)  
    

print(map_origin_pixel)
print((1707183149 - 1707181398)/60)
print(image.shape)
cv2.imwrite("src/mobot_pkg/data/Map_Annotated_path.png", image)


        
        


with open("src/mobot_pkg/data/base_footprint_points.json", 'w', encoding="utf-8") as wf:
    json.dump(Datas, wf)



