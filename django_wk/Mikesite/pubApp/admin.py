from django.contrib import admin
from pubApp.models import Paper
from pubApp.models import Bio
from pubApp.models import Text

# Register your models here.
admin.site.register(Paper)
admin.site.register(Bio)
admin.site.register(Text)