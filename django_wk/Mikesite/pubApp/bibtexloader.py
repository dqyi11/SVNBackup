'''
Created on 2013-12-27

@author: Walter
'''
from bibtexparser.bparser import BibTexParser
from models import Paper

class BibtexLoader(object):
    '''
    classdocs
    '''
    def __init__(self, bibtex_filename):
        '''
        Constructor
        '''
        self.filename = bibtex_filename
        self.entry_list = []
        with open(self.filename) as bibfile:
            bp = BibTexParser(bibfile)
            self.entry_list = bp.get_entry_list()
            
    def write_db(self):
        
        print "len of entry list " + str(len(self.entry_list))
        
        for entry in self.entry_list:
            paper = Paper()
            if entry.has_key("id"):
                paper.id = entry["id"]
            if entry.has_key("type"):
                paper.type = entry["type"]
            if entry.has_key("title"):
                paper.title = entry["title"]
            if entry.has_key("author"):
                paper.authors = entry["author"]
            if entry.has_key("year"):
                paper.year = int(entry["year"])
            if entry.has_key("journal"):
                paper.journal = entry["journal"]
            if entry.has_key("booktitle"):
                paper.book_title = entry["booktitle"]
            if entry.has_key("publisher"):
                paper.publisher = entry["publisher"]
            if entry.has_key("institution"):
                paper.institution = entry["institution"]
            if entry.has_key("volume"):
                paper.volume = int(entry["volume"])
            if entry.has_key("number"):
                paper.number = int(entry["number"])
            if entry.has_key("pages"):
                paper.pages = entry["pages"]
            if entry.has_key("url"):
                paper.url = entry["url"]
            if entry.has_key("doi"):
                paper.doi = entry["doi"]
            if entry.has_key("isbn"):
                paper.isbn = entry["isbn"]
                
            paper.save()
            

                
                
        
        
            
    
        
    
        