

def replaceFileHead(filename, outputFilename):
    with open(outputFilename, "wt") as fout:
        with open(filename, "rt") as fin:
            context = fin.read()
            context = context.replace('<?xml version="1.0" ?>','')
            context = context.replace('<World>', '<World xmlns="VisibilityGraph" xmlns:i="http://www.w3.org/2001/XMLSchema-instance">')
            #context = context.replace(' ', '')
            #context = context.replace('\n','')
            fout.write(context)
            