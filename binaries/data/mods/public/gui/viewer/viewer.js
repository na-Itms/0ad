
/* Globals */
var g_SelectedCiv = "gaia"; // fallback default
var g_CallbackSet = false;

/**
 * Init. Also populates the gui objects.
 * 
 * @arg template The object or the template name of the entity to be displayed
 */
function init (template = null) {
	if (!template)
	{
		error("Viewer: No template provided");
		closeViewer();
		return;
	}
	else if (typeof template === "string" || template.entityName)
	{
		var templateName = template.entityName || template;
		if (template.callback)
			g_CallbackSet = true;
		if (template.civ)
			g_SelectedCiv = template.civ;

		template = loadTemplateFromName(templateName);
		if (!template)
		{
			error("Viewer: unable to recognise or load template: "+templateName);
			closeViewer();
			return;
		}
	}

	Engine.GetGUIObjectByName("entityName").caption = getEntityNamesFormatted(template);
	Engine.GetGUIObjectByName("entityIcon").sprite = "stretched:session/portraits/" + template.icon;

	var caption = "";
	if (template.cost)
		caption += getEntityCostTooltip(template, 1) + "\n";
	Engine.GetGUIObjectByName("entityStats").caption = caption + getEntityStats(template);

	var txt = "";

	if (template.tooltip)
		txt += "\n" + g_TooltipTextFormats.body[0] +  translate(template.tooltip) + g_TooltipTextFormats.body[1] + "\n";

	if (template.history)
		txt += "\n" + g_TooltipTextFormats.body[0] +  translate(template.history) + g_TooltipTextFormats.body[1] + "\n";

	if (template.description)
		txt += "\n" + g_TooltipTextFormats.body[0] +  translate(template.description) + g_TooltipTextFormats.body[1] + "\n";

	if (template.auras)
		txt += getAurasTooltip(template) + "\n";
	
	txt += getVisibleEntityClassesFormatted(template);

	Engine.GetGUIObjectByName("entityInfo").caption = txt;
}

/**
 * Determines the requested template and loads it
 * 
 * @arg templateName The template name. If loading a technology, then `tech/` must be prefixed.
 * @return The entity object if successful, false if not
 */
function loadTemplateFromName(templateName)
{
	if (templateName.indexOf("|") > -1)
		templateName = templateName.slice(templateName.indexOf("|")+1);
	var prefix = templateName.slice(0, templateName.indexOf("/"));

	switch (prefix)
	{
	case "structures":
	case "other":
		return loadStructure(templateName);
		break;

	case "units":
		return loadUnit(templateName);
		break;

	case "gaia":
		return loadResource(templateName);
		break;

	case "tech":
		return loadTechnology(templateName.substr(templateName.indexOf("/")+1));
		break;

	default:
		// do nothing (error message is given elsewhere)
	}

	return false;
}

/**
 * Overrides near-identical function in gui/common/tooltips.js
 * (Just so we can get slightly bigger text)
 */
function getEntityNamesFormatted(template)
{
	var names = "";
	var generic = template.name.generic;
	var specific = template.name.specific;
	if (specific)
	{
		// drop caps for specific name
		names += '[font="sans-bold-20"]' + specific[0] + '[/font]' +
			'[font="sans-bold-16"]' + specific.slice(1).toUpperCase() + '[/font]';

		if (generic)
			names += '[font="sans-bold-16"] (' + generic + ')[/font]';
	}
	else if (generic)
		names = '[font="sans-bold-20"]' + generic + "[/font]";
	else
		names = "???";

	return names;
}

function loadResource(templateName)
{
	var template = loadTemplate(templateName);
	var resource = GetTemplateDataHelper(template);

	resource.history = template.Identity.History;

	resource.supply = {
		"type": template.ResourceSupply.Type.split("."),
		"amount": template.ResourceSupply.Amount,
	};

	return resource;
}

/**
 * Closes the page
 */
function closeViewer() 
{ 
	if (g_CallbackSet)
		Engine.PopGuiPageCB(0); 
	else 
		Engine.PopGuiPage(); 
}
