from django.shortcuts import render
from django.http import HttpResponse
from django.core.files import File
from Mikesite import settings
from bibtexloader import BibtexLoader
from pubApp.models import Paper
from pubApp.models import Bio
from pubApp.models import Text
from pubApp.pubmanager import PubManager
from django.shortcuts import render_to_response
import sys

bibfile = settings.REFERENCE_BIB_FILE


# Create your views here.
def papers(request):
    
    strOut = ""
    #for entry in entry_list:
    #    strOut += str(entry)    
    return HttpResponse(strOut)

def home(request):
    bios = Bio.objects.all()
    bio = bios[0]
    
    return render_to_response('home.html', {
        'bio': bio,
        'page_width': 900,
    })   

def teaching(request):
    bio = Bio.objects.all()[0]
    teaching = Text.objects.filter(title='Teaching')[0]
    
    return render_to_response('teaching.html', {
        'bio': bio,
        'text': teaching,
        'page_width': 900,
    })
    
def load(request):
    
    try:
        loader = BibtexLoader(bibfile)
    except:
        e = sys.exc_info()[0]
        return HttpResponse("<p>Error: %s</p>"%e)
    
    loader.write_db()
    return HttpResponse("<p>Success!</p>")

def all(request):
    bio = Bio.objects.all()[0]
    
    pubMgr = PubManager()
    pub_years = pubMgr.getPubList()
    year_list = pubMgr.getYearList()
 
    return render_to_response('all.html', {
        'pub_years': pub_years,
        'year_list' : year_list,
        'page_width': 900,
        'bio': bio,
        'year_idx' : str(pubMgr.getYearIndex()),
        'type_idx' : pubMgr.getTypeIndex(),
    })
        
        
def search(request):
    
    type = request.GET.get('type')
    year = request.GET.get('year')
    
    bio = Bio.objects.all()[0]
    pubMgr = PubManager()
    if type != None:
        trans_type = "all"
        type = type.lower()
        if type == "journal":
            trans_type = "article"
        elif type == "conference":
            trans_type = "inproceedings"
        pubMgr.type = trans_type
        
    if year != None:
        if year != "all":
            pubMgr.year = year
        else:
            year = year.lower()
                
    pub_years = pubMgr.getPubList()
    year_list = pubMgr.getYearList()
    
    return render_to_response('all.html', {
        'pub_years': pub_years,
        'year_list' : year_list,
        'page_width': 900,
        'bio': bio,
        'year_idx' : str(pubMgr.getYearIndex()),
        'type_idx' : pubMgr.getTypeIndex(),
    })        
    
