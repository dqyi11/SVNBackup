'''
Created on 2013-12-29

@author: Walter
'''

from pubApp.models import Paper

class PubManager(object):
    '''
    classdocs
    '''


    def __init__(self):
        '''
        Constructor
        '''
        self.type = "all"
        self.year = "all"
        
        self.type_all = "all"
        self.type_article = "article"
        self.type_proceeding = "inproceedings"
        self.year_all = "all"
        
        self.typeIdx = 0
        
    def getYearList(self):       
        years = []        
        publications = Paper.objects.all();
        for p in publications:
            if not (str(p.year) in years):
                years.append(str(p.year))
        years.sort(reverse=True)
        return years
        
    def getPubList(self):   
        papers = []
        if self.type == "all" and self.year =="all":
            papers = Paper.objects.all().order_by('-year','title')
        elif self.type == "all":
            papers = Paper.objects.filter(year=self.year).order_by('-year','title')
        elif self.year == "all":
            papers = Paper.objects.filter(type=self.type).order_by('-year','title')
        else:
            papers = Paper.objects.filter(type=self.type).filter(year=self.year).order_by('-year','title')
            
        self.typeIdx = self.getTypeIndex()
            
        pub_years = []
        for p in papers:
            ys = [y for y in pub_years if p.year==y[0]]
            if len(ys) == 0:
                pub_years.append((p.year, []))
        
        for p in papers:
            year = next(y for y in pub_years if p.year==y[0])
            year[1].append(p)
            
        return pub_years
    
    
    def getTypeIndex(self):

        if self.type == self.type_article:
            return 1
        if self.type == self.type_proceeding:
            return 2
        
        return 0
        
    def getYearIndex(self):
        
        if self.year == self.year_all:
            return 0
        
        return int(self.year)
        
        
        
        