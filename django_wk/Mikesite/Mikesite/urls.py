from django.conf.urls import patterns, include, url
from django.contrib import admin
from Mikesite import settings
admin.autodiscover()

urlpatterns = patterns('',
    # Examples:
    # url(r'^$', 'Mikesite.views.home', name='home'),
    # url(r'^blog/', include('blog.urls')),
    url(r'^$', 'pubApp.views.home', name='home'),
    url(r'^papers/$', 'pubApp.views.all'),
    url(r'^papers/search/', 'pubApp.views.search'),
    url(r'^teaching/$', 'pubApp.views.teaching'),
    url(r'^load/$', 'pubApp.views.load', name='Load Bibtex'),
    url(r'^admin/', include(admin.site.urls)),    
    url( r'^static/(?P<path>.*)$', 'django.views.static.serve', { 'document_root':settings.STATIC_ROOT, 'show_indexes': True }),
)
