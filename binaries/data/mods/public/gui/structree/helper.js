var g_TemplateData = {};
var g_TechnologyData = {};

function loadTemplate(templateName)
{
	if (!(templateName in g_TemplateData))
	{
		// We need to clone the template because we want to perform some translations.
		var data = clone(Engine.GetTemplate(templateName));
		translateObjectKeys(data, ["GenericName", "SpecificName", "Tooltip", "History"]);
		
		g_TemplateData[templateName] = data;
	}

	return g_TemplateData[templateName];
}

function loadTechData(templateName)
{
	if (!(templateName in g_TechnologyData))
	{
		var filename = "simulation/data/technologies/" + templateName + ".json";
		var data = Engine.ReadJSONFile(filename);
		translateObjectKeys(data, ["genericName", "tooltip"]);
		
		g_TechnologyData[templateName] = data;
	}

	return g_TechnologyData[templateName];
}

/**
 * Fetch a value from an entity's template
 *
 * @param templateName The template to retreive the value from
 * @param keypath The path to the value to be fetched. "Identity/GenericName"
 *                is equivalent to {"Identity":{"GenericName":"FOOBAR"}}
 *
 * @return The content requested at the key-path defined, or a blank array if
 *           not found
 */
function fetchValue(templateName, keypath)
{
	var keys = keypath.split("/");
	var template = loadTemplate(templateName);

	let k = 0;
	for (; k < keys.length-1; ++k)
	{
		if (template[keys[k]] === undefined)
			return [];

		template = template[keys[k]];
	}
	if (template[keys[k]] === undefined)
		return [];

	return template[keys[k]];
}

/**
 * Fetch tokens from an entity's template
 * @return An array containing all tokens if found, else an empty array
 * @see fetchValue
 */
function fetchTokens(templateName, keypath)
{
	var val = fetchValue(templateName, keypath);
	if (!("_string" in val))
		return [];

	return val._string.split(" ");
}

function depath(path)
{
	return path.slice(path.lastIndexOf("/")+1);
}

/**
 * This is needed because getEntityCostTooltip in tooltip.js needs to get
 * the template data of the different wallSet pieces. In the session this
 * function does some caching, but here we do that in loadTemplate already.
 */
function GetTemplateData(templateName)
{
	var template = loadTemplate(templateName);
	return GetTemplateDataHelper(template);
}

function getEntityStats(template)
{
	var txt = "";

	if (template.health)
		txt += "\n" + sprintf(translate("%(label)s %(details)s"), {
			label: g_TooltipTextFormats.header[0] + translate("Health:") + g_TooltipTextFormats.header[1],
			details: template.health
		});

	if (template.healer)
		txt += "\n" + getHealerTooltip(template);

	if (template.attack)
		txt += "\n" + getAttackTooltip(template);

	if (template.armour)
		txt += "\n" + getArmorTooltip(template.armour);

	if (template.speed)
		txt += "\n" + getSpeedTooltip(template);

	if (template.gather)
	{
		var rates = [];
		for (let type in template.gather)
			rates.push(sprintf(translate("%(resourceIcon)s %(rate)s"), {
				resourceIcon: getCostComponentDisplayName(type),
				rate: template.gather[type]
			}));

		txt += "\n" + sprintf(translate("%(label)s %(details)s"), {
			label: g_TooltipTextFormats.header[0] + translate("Gather Rates:") + g_TooltipTextFormats.header[1],
			details: rates.join("  ")
		});
	}

	if (template.supply)
		txt += "\n" + getResourceSupplyTooltip(template);

	return txt;
}
