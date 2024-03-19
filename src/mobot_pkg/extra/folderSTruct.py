

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
            if ".pyc" not in pth.lower() and "git" not in pth.lower():
                mdLines.append(f"{prefix}- {pth}")
    return mdLines

def main():
    rootDir = '/home/shihab/Downloads/omobot_js/'  # Change this to your directory path
    mdTree = generateTree2MD(rootDir)
    mdOutput = '\n'.join(mdTree)
    print(mdOutput)
    # Optionally, write to a Markdown file
    with open('src/mobot_pkg/extra/folderTree', 'w') as wf:
        wf.write(mdOutput)

if __name__ == '__main__':
    main()
