'''
Created on Dec 27, 2013

@author: daqing_yi
'''

from bibtexparser.bparser import BibTexParser

if __name__ == '__main__':
    
    with open('reference.bib') as bibfile:
        bp = BibTexParser(bibfile)
        entry_list = bp.get_entry_list()
        
        print len(entry_list)
        '''
        for entry in entry_list:
            print entry["id"]
            print entry["type"]
            print entry["author"]
            #print entry["institution"]
            print entry["title"]
            print entry["year"]
        '''