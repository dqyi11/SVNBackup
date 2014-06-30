from django.db import models

# Create your models here.
class Paper(models.Model):
    id = models.CharField(max_length=256, primary_key=True, unique=True)
    type = models.CharField(max_length=32)
    title = models.CharField(max_length=256)
    authors = models.CharField(max_length=256)
    year = models.PositiveIntegerField(max_length=4)
    journal = models.CharField(max_length=256, blank=True)
    book_title = models.CharField(max_length=256, blank=True)
    publisher = models.CharField(max_length=256, blank=True)
    institution = models.CharField(max_length=256, blank=True)
    volume = models.CharField(max_length=128, blank=True, null=True)
    number = models.CharField(max_length=128, blank=True, null=True)
    pages = models.CharField(max_length=32, blank=True)
    url = models.URLField(blank=True)
    doi = models.CharField(max_length=128, blank=True)
    isbn = models.CharField(max_length=32, blank=True)
    

class Bio(models.Model):
    full_name = models.CharField(max_length=128)
    title = models.CharField(max_length=32)
    photo = models.URLField(max_length=512)
    school = models.CharField(max_length=128)
    department = models.CharField(max_length=128)
    address = models.CharField(max_length=512)
    phone = models.CharField(max_length=32)
    email = models.EmailField(max_length=128)
    fax = models.CharField(max_length=32)
    cv = models.URLField(max_length=512, blank=True, null=True)
    wiki = models.URLField(max_length=512, blank=True, null=True)
    acm = models.URLField(max_length=512, blank=True, null=True)
    google_scholar = models.URLField(max_length=512, blank=True, null=True)
    

class Text(models.Model):
    title = models.CharField(max_length=512)
    author = models.CharField(max_length=128)
    abstract = models.CharField(max_length=2048)
    context = models.CharField(max_length=10240)
    mod_date = models.DateField(auto_now=True)
    mod_time = models.TimeField(auto_now=True)