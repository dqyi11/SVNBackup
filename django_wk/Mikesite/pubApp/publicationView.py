'''
Created on 2013-12-27

@author: Walter
'''

from django.template import Context
from django.template.loader import get_template
from pubApp.models import Paper

def get_publication_view(id):
    
    pub = Paper.objects.filter(id)
    
    if len(pub) < 1:
        return ""
    
    return get_template('templates/publication.html').render(Context({'publication': pub[0]}))
    
    
    
    