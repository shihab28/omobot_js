

import os

def generateTree2MD(rootdir, prefix=''):
    paths = os.listdir(rootdir)
    paths.sort()
    mdLines = []
    for pth in paths:
        path = os.path.join(rootdir, pth)
        if os.path.isdir(path):
            mdLines.append(f"{prefix}+ {pth}/")
            mdLines += generateTree2MD(path, prefix + "    ")
        else:
            mdLines.append(f"{prefix}- {pth}")
    return mdLines

def main():
    rootDir = '/home/shihab/Downloads/omobotBackup'  # Change this to your directory path
    mdTree = generateTree2MD(rootDir)
    mdOutput = '\n'.join(mdTree)
    print(mdOutput)
    # Optionally, write to a Markdown file
    # with open('directory_structure.md', 'w') as md_file:
    #     md_file.write(markdown_output)

if __name__ == '__main__':
    main()
