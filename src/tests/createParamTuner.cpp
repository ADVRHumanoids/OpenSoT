/*
 * Copyright: (C) 2014 Walkman Consortium
 * Authors: Alessio Rocchi
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
*/


#include "sot_VelKinCon_constants.h"
#include <string>
#include <math.h>
#include <assert.h>

/** creates a yarpscope xml file starting from ParamProxyInterface array */
const std::string createParamTuner(const ParamProxyInterface *const sot_VelKinCon_ParamDescr[], unsigned int size, std::string moduleName);

int main(int argc, char* argv[]) {
    std::string moduleName = std::string("sot_VelKinCon");
    const std::string scopeXml = createParamTuner(wb_sot::sot_VelKinCon_ParamDescr,
                                                  wb_sot::PARAM_ID_SIZE,
                                                  moduleName);
    std::cout << scopeXml;
    return 0;
}

/** takes an array of ParamProxies, creates a scope for them
    TODO move this to a proper place
    TODO make this somehow automatic (e.g. a program using paramHelp
         will be automatically be able to create the monitor scope when calling
         paramHelp with a script, or maybe with some util?*/
const std::string createParamTuner(const ParamProxyInterface *const sot_VelKinCon_ParamDescr[],
                                   unsigned int size,
                                   std::string moduleName) {
    std::stringstream preamble;
    std::stringstream postamble;
    std::stringstream adjustments;
    std::stringstream main;

    preamble << "\
<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n\
<interface>\n\
<!-- interface-requires gtk+ 3.0 -->"<< std::endl;

    main << "\
  <object class=\"GtkWindow\" id=\"parameterTuning\">\n\
    <property name=\"width_request\">300</property>\n\
    <property name=\"can_focus\">False</property>\n\
    <property name=\"title\" translatable=\"yes\">parameterTuning</property>\n\
    <property name=\"resizable\">False</property>\n\
    <signal name=\"delete-event\" handler=\"onDeleteWindow\" swapped=\"no\"/>\n\
    <child>\n\
      <object class=\"GtkFrame\" id=\"main_frame\">\n\
        <property name=\"visible\">True</property>\n\
        <property name=\"can_focus\">False</property>\n\
        <property name=\"label_xalign\">0</property>\n\
        <property name=\"shadow_type\">none</property>\n\
        <child>\n\
          <object class=\"GtkAlignment\" id=\"frame_alignment\">\n\
            <property name=\"visible\">True</property>\n\
            <property name=\"can_focus\">False</property>\n\
            <property name=\"left_padding\">12</property>\n\
            <child>\n\
              <object class=\"GtkBox\" id=\"main_hbox\">\n\
                <property name=\"visible\">True</property>\n\
                <property name=\"can_focus\">False</property>\n\
                <property name=\"orientation\">vertical</property>\n\
                <property name=\"spacing\">5</property>" << std::endl;

    for(unsigned int i = 0; i < size; ++i) {
        const ParamProxyInterface *proxy = sot_VelKinCon_ParamDescr[i];
        if(proxy->ioType.value == paramHelp::PARAM_IN_OUT) {

            if(dynamic_cast<const paramHelp::ParamProxyBasic<double>* >(proxy) != NULL) {
                unsigned int displacement = 0;
                main << "\n\
                <child>\n\
                  <object class=\"GtkExpander\" id=\"expander_" << proxy->id <<"\">\n\
                    <property name=\"visible\">True</property>\n\
                      <property name=\"can_focus\">False</property>\n\
                      <property name=\"tooltip_text\" translatable=\"yes\">" << proxy->description << "</property>\n\
                      <property name=\"expanded\">True</property>\n\
                      <child>\n\
                        <object class=\"GtkBox\" id=\"vbox_" <<  proxy->id << "\">\n\
                          <property name=\"visible\">True</property>\n\
                          <property name=\"can_focus\">False</property>\n\
                          <property name=\"orientation\">vertical</property>" << std::endl;

/*********************** DOUBLE VALUES, SINGLE ELEMENT: SLIDER + SPINBOX **********************/
                if(proxy->size == 1) {
                    adjustments << "\
  <object class=\"GtkAdjustment\" id=\"adjustment_"<< proxy->id + displacement <<"\">" << std::endl;
                    /** @TODO add limits */
                    adjustments << "\
    <property name=\"upper\">100</property>\n\
    <property name=\"step_increment\">0.1</property>\n\
    <property name=\"page_increment\">10</property>\n\
  </object>" << std::endl;

                    main << "\
                      <child>\n\
                        <object class=\"GtkBox\" id=\"hbox_" << proxy->id + displacement << "\">\n\
                          <property name=\"visible\">True</property>\n\
                          <property name=\"can_focus\">False</property>\n\
                          <child>\n\
                            <object class=\"GtkScale\" id=\"scale_" << proxy->id + displacement << "\">\n\
                              <property name=\"width_request\">200</property>\n\
                              <property name=\"visible\">True</property>\n\
                              <property name=\"can_focus\">False</property>\n\
                              <property name=\"adjustment\">adjustment_" << proxy->id + displacement << "</property>\n\
                              <property name=\"restrict_to_fill_level\">False</property>\n\
                              <property name=\"fill_level\">0</property>\n\
                              <property name=\"round_digits\">1</property>\n\
                              <property name=\"digits\">1</property>\n\
                              <property name=\"value_pos\">bottom</property>\n\
                            </object>\n\
                            <packing>\n\
                              <property name=\"expand\">False</property>\n\
                              <property name=\"fill\">True</property>\n\
                              <property name=\"padding\">5</property>\n\
                              <property name=\"position\">0</property>\n\
                            </packing>\n\
                          </child>\n\
                          <child>\n\
                            <object class=\"GtkSpinButton\" id=\"spinbutton_" << proxy->id + displacement << "\">\n\
                              <property name=\"width_request\">90</property>\n\
                              <property name=\"visible\">True</property>\n\
                              <property name=\"can_focus\">True</property>\n\
                              <property name=\"invisible_char\">•</property>\n\
                              <property name=\"input_purpose\">digits</property>\n\
                              <property name=\"adjustment\">adjustment_" << proxy->id + displacement << "</property>\n\
                              <property name=\"digits\">1</property>\n\
                              <property name=\"snap_to_ticks\">True</property>\n\
                              <property name=\"numeric\">True</property>\n\
                              <property name=\"update_policy\">if-valid</property>\n\
                              <signal name=\"value-changed\" handler=\"onValueChangedDouble\" swapped=\"no\"/>\n\
                            </object>\n\
                            <packing>\n\
                              <property name=\"expand\">False</property>\n\
                              <property name=\"fill\">True</property>\n\
                              <property name=\"position\">1</property>\n\
                            </packing>\n\
                          </child>\n\
                        </object>\n\
                        <packing>\n\
                          <property name=\"expand\">False</property>\n\
                          <property name=\"fill\">True</property>\n\
                          <property name=\"padding\">5</property>\n\
                          <property name=\"position\">1</property>\n\
                        </packing>\n\
                      </child>"         << std::endl;

/*********************** DOUBLE VALUES, MULTIPLE ELEMENTS: SLIDER + SPINBOX **********************/
                } else if(proxy->size > 1) {
                      ; /**  @TODO UNSUPPORTED */
                }

                main << "\
                        </object>\n\
                      </child>\n\
                      <child type=\"label\">\n\
                        <object class=\"GtkLabel\" id=\"expander_" << proxy->id << "_label\">\n\
                          <property name=\"visible\">True</property>\n\
                          <property name=\"can_focus\">False</property>\n\
                          <property name=\"tooltip_text\" translatable=\"yes\">" << proxy->description << "</property>\n\
                          <property name=\"label\" translatable=\"yes\">" << proxy->name << "</property>\n\
                        </object>\n\
                      </child>\n\
                    </object>\n\
                    <packing>\n\
                      <property name=\"expand\">False</property>\n\
                      <property name=\"fill\">True</property>\n\
                      <property name=\"padding\">5</property>\n\
                      <property name=\"position\">0</property>\n\
                    </packing>\n\
                  </child>" << std::endl;

            } else if(dynamic_cast<const paramHelp::ParamProxyBasic<bool>* >(proxy) != NULL) {

                 unsigned int displacement = 0;
                 main << "\n\
                 <child>\n\
                   <object class=\"GtkExpander\" id=\"expander_" << proxy->id <<"\">\n\
                     <property name=\"visible\">True</property>\n\
                       <property name=\"can_focus\">False</property>\n\
                       <property name=\"tooltip_text\" translatable=\"yes\">" << proxy->description << "</property>\n\
                       <property name=\"expanded\">True</property>\n\
                       <child>\n\
                         <object class=\"GtkBox\" id=\"vbox_" <<  proxy->id << "\">\n\
                           <property name=\"visible\">True</property>\n\
                           <property name=\"can_focus\">False</property>\n\
                           <property name=\"orientation\">vertical</property>" << std::endl;

 /*********************** BOOLEAN VALUES, SINGLE ELEMENT: CHECKBOX **********************/
                 if(proxy->size == 1) {

                     main << "\
                           <child>\n\
                             <object class=\"GtkCheckButton\" id=\"checkbutton_" << proxy->id + displacement << "\">\n\
                               <property name=\"label\" translatable=\"yes\">" << proxy->name << "</property>\n\
                               <property name=\"tooltip_text\" translatable=\"yes\">" << proxy->description << "</property>\n\
                               <property name=\"visible\">True</property>\n\
                               <property name=\"can_focus\">False</property>\n\
                               <property name=\"receives_default\">False</property>\n\
                               <property name=\"xalign\">0</property>\n\
                               <property name=\"draw_indicator\">True</property>\n\
                               <signal name=\"toggled\" handler=\"onValueChangedBool\" swapped=\"no\"/>\n\
                             </object>\n\
                             <packing>\n\
                               <property name=\"expand\">False</property>\n\
                               <property name=\"fill\">True</property>\n\
                               <property name=\"position\">0</property>\n\
                             </packing>\n\
                           </child>"         << std::endl;

 /*********************** BOOLEAN VALUES, MULTIPLE ELEMENTS: CHECKBOXES **********************/
                 } else if(proxy->size > 1) {
                       ; /**  @TODO UNSUPPORTED */
                 }

                 main << "\
                        </object>\n\
                       </child>\n\
                       <child type=\"label\">\n\
                         <object class=\"GtkLabel\" id=\"expander_" << proxy->id << "_label\">\n\
                           <property name=\"visible\">True</property>\n\
                           <property name=\"can_focus\">False</property>\n\
                           <property name=\"tooltip_text\" translatable=\"yes\">" << proxy->description << "</property>\n\
                           <property name=\"label\" translatable=\"yes\">" << proxy->name << "</property>\n\
                         </object>\n\
                       </child>\n\
                     </object>\n\
                     <packing>\n\
                       <property name=\"expand\">False</property>\n\
                       <property name=\"fill\">True</property>\n\
                       <property name=\"padding\">5</property>\n\
                       <property name=\"position\">0</property>\n\
                     </packing>\n\
                   </child>" << std::endl;

            } else if(dynamic_cast<const paramHelp::ParamProxyBasic<int>* >(proxy) != NULL) {
                 unsigned int displacement = 0;
                 main << "\n\
                 <child>\n\
                   <object class=\"GtkExpander\" id=\"expander_" << proxy->id <<"\">\n\
                     <property name=\"visible\">True</property>\n\
                       <property name=\"can_focus\">False</property>\n\
                       <property name=\"tooltip_text\" translatable=\"yes\">" << proxy->description << "</property>\n\
                       <property name=\"expanded\">True</property>\n\
                       <child>\n\
                         <object class=\"GtkBox\" id=\"vbox_" <<  proxy->id << "\">\n\
                           <property name=\"visible\">True</property>\n\
                           <property name=\"can_focus\">False</property>\n\
                           <property name=\"orientation\">vertical</property>" << std::endl;

 /*********************** INTEGER VALUES, SINGLE ELEMENT: SLIDER + SPINBOX **********************/
                 if(proxy->size == 1) {
                     adjustments << "\
   <object class=\"GtkAdjustment\" id=\"adjustment_"<< proxy->id + displacement <<"\">" << std::endl;
                     /** @TODO add limits */
                     adjustments << "\
     <property name=\"upper\">100</property>\n\
     <property name=\"step_increment\">1</property>\n\
     <property name=\"page_increment\">10</property>\n\
   </object>" << std::endl;

                     main << "\
                       <child>\n\
                         <object class=\"GtkBox\" id=\"hbox_" << proxy->id + displacement << "\">\n\
                           <property name=\"visible\">True</property>\n\
                           <property name=\"can_focus\">False</property>\n\
                           <child>\n\
                             <object class=\"GtkScale\" id=\"scale_" << proxy->id + displacement << "\">\n\
                               <property name=\"width_request\">200</property>\n\
                               <property name=\"visible\">True</property>\n\
                               <property name=\"can_focus\">False</property>\n\
                               <property name=\"adjustment\">adjustment_" << proxy->id + displacement << "</property>\n\
                               <property name=\"restrict_to_fill_level\">False</property>\n\
                               <property name=\"fill_level\">0</property>\n\
                               <property name=\"round_digits\">0</property>\n\
                               <property name=\"digits\">0</property>\n\
                               <property name=\"value_pos\">bottom</property>\n\
                             </object>\n\
                             <packing>\n\
                               <property name=\"expand\">False</property>\n\
                               <property name=\"fill\">True</property>\n\
                               <property name=\"padding\">5</property>\n\
                               <property name=\"position\">0</property>\n\
                             </packing>\n\
                           </child>\n\
                           <child>\n\
                             <object class=\"GtkSpinButton\" id=\"spinbutton_" << proxy->id + displacement << "\">\n\
                               <property name=\"width_request\">90</property>\n\
                               <property name=\"visible\">True</property>\n\
                               <property name=\"can_focus\">True</property>\n\
                               <property name=\"invisible_char\">•</property>\n\
                               <property name=\"input_purpose\">digits</property>\n\
                               <property name=\"adjustment\">adjustment_" << proxy->id + displacement << "</property>\n\
                               <property name=\"digits\">0</property>\n\
                               <property name=\"snap_to_ticks\">True</property>\n\
                               <property name=\"numeric\">True</property>\n\
                               <property name=\"update_policy\">if-valid</property>\n\
                               <signal name=\"value-changed\" handler=\"onValueChangedInt\" swapped=\"no\"/>\n\
                             </object>\n\
                             <packing>\n\
                               <property name=\"expand\">False</property>\n\
                               <property name=\"fill\">True</property>\n\
                               <property name=\"position\">1</property>\n\
                             </packing>\n\
                           </child>\n\
                         </object>\n\
                         <packing>\n\
                           <property name=\"expand\">False</property>\n\
                           <property name=\"fill\">True</property>\n\
                           <property name=\"padding\">5</property>\n\
                           <property name=\"position\">1</property>\n\
                         </packing>\n\
                       </child>"         << std::endl;

 /*********************** INTEGER VALUES, MULTIPLE ELEMENTS: SLIDER + SPINBOX **********************/
                 } else if(proxy->size > 1) {
                       ; /**  @TODO UNSUPPORTED */
                 }

                 main << "\
                         </object>\n\
                       </child>\n\
                       <child type=\"label\">\n\
                         <object class=\"GtkLabel\" id=\"expander_" << proxy->id << "_label\">\n\
                           <property name=\"visible\">True</property>\n\
                           <property name=\"can_focus\">False</property>\n\
                           <property name=\"tooltip_text\" translatable=\"yes\">" << proxy->description << "</property>\n\
                           <property name=\"label\" translatable=\"yes\">" << proxy->name << "</property>\n\
                         </object>\n\
                       </child>\n\
                     </object>\n\
                     <packing>\n\
                       <property name=\"expand\">False</property>\n\
                       <property name=\"fill\">True</property>\n\
                       <property name=\"padding\">5</property>\n\
                       <property name=\"position\">0</property>\n\
                     </packing>\n\
                   </child>" << std::endl;

            } else
                ; // UNSUPPORTED TYPE

        }
    }

    main     << "\
              </object>\n\
            </child>\n\
          </object>\n\
        </child>\n\
      </object>\n\
    </child>\n\
  </object>" << std::endl;
    postamble << "\
</interface>"<< std::endl;

    preamble.sync();
    adjustments.sync();
    main.sync();
    postamble.sync();
    return preamble.str() + adjustments.str() + main.str();
}

