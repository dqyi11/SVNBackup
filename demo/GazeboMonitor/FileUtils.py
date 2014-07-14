

def replaceFileHead(filename, outputFilename):
    with open(outputFilename, "wt") as fout:
        with open(filename, "rt") as fin:
            for line in fin:
                fout.write(line.replace('A', 'Orange'))