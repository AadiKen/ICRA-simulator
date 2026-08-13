import{readFileSync}from"node:fs";import{dirname,join}from"node:path";import{fileURLToPath}from"node:url";
const root=dirname(fileURLToPath(import.meta.url));
export const loadCase=(id)=>JSON.parse(readFileSync(join(root,"cases",`${id}.json`),"utf8"));
export const CASE_IDS=["marin_l7_experiment","yasukawa_l7_unresolved","l3_coefficient_diagnostic"];
export function validateCase(config){
  if(config.schema_version!==1||config.id!==config.operating_point.configuration)throw new Error("Operating-point identity mismatch.");
  if(!["experimental_validation","published_result_comparison","parameter_reproduction","diagnostic","sensitivity_analysis","blocked_missing_primary_source"].includes(config.validation_label))throw new Error(`Invalid validation label for ${config.id}.`);
  if(config.applicable_configuration!==config.model_parameters.allowed_for_case)throw new Error(`${config.id} mixes ${config.applicable_configuration} case data with a model resolved for ${config.model_parameters.allowed_for_case}.`);
  if(!config.model_parameters.geometry_configuration||!config.model_parameters.coefficient_configuration)throw new Error(`${config.id} must identify geometry and coefficient configurations separately.`);
  if(config.validation_label==="blocked_missing_primary_source"){
    const unresolved=Object.entries(config.operating_point).filter(([,value])=>value===null).map(([key])=>key);
    if(unresolved.length)throw new Error(`${config.id}: blocked_missing_primary_source: supply ${unresolved.join(", ")} in cases/${config.id}.json.`);
  }
  for(const [name,parameter] of Object.entries(config.operating_point))if(name!=="configuration"&&parameter!==null&&typeof parameter==="object"&&!Array.isArray(parameter)){for(const field of ["value","units","source_document","location","status","applicable_configuration"])if(!(field in parameter))throw new Error(`${config.id}.${name} lacks provenance field ${field}.`);if(parameter.applicable_configuration!==config.applicable_configuration)throw new Error(`${config.id}.${name} mixes configuration provenance.`);}
  return config;
}
