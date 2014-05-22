/*
 * Copyright: (C) 2014 Walkman Consortium
 * Authors: Alessio Rocchi
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
*/


#include "sot_VelKinCon_constants.h"
#include <string>
#include <math.h>
#include <assert.h>

#define DEFAULT_UPPER_BOUND 65532.0
#define DEFAULT_LOWER_BOUND -65532.0

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
    @todo move this to a proper place (e.g. drc_shared)
    @todo make this somehow automatic (e.g. a program using paramHelp
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
    <property name=\"width_request\">350</property>\n\
    <property name=\"height_request\">500</property>\n\
    <property name=\"can_focus\">False</property>\n\
    <property name=\"title\" translatable=\"yes\">parameterTuning</property>\n\
    <property name=\"resizable\">True</property>\n\
    <signal name=\"delete-event\" handler=\"onDeleteWindow\" swapped=\"no\"/>\n\
    <child>\n\
      <object class=\"GtkScrolledWindow\" id=\"main_scrolled\">\n\
        <property name=\"visible\">True</property>\n\
        <property name=\"can_focus\">False</property>\n\
        <property name=\"shadow_type\">none</property>\n\
        <child>\n\
          <object class=\"GtkViewport\" id=\"main_viewport\">\n\
            <property name=\"visible\">True</property>\n\
            <property name=\"can_focus\">False</property>\n\
            <property name=\"left_padding\">12</property>\n\
            <property name=\"halign\">start</property>\n\
            <property name=\"valign\">start</property>\n\
            <child>\n\
              <object class=\"GtkBox\" id=\"main_hbox\">\n\
                <property name=\"visible\">True</property>\n\
                <property name=\"can_focus\">False</property>\n\
                <property name=\"orientation\">vertical</property>\n\
                <property name=\"spacing\">5</property>\n\
                <property name=\"margin_left\">10</property>\n\
                <property name=\"halign\">start</property>\n\
                <property name=\"valign\">start</property>" << std::endl;

    for(unsigned int i = 0; i < size; ++i) {
        const ParamProxyInterface *proxy = sot_VelKinCon_ParamDescr[i];
        if(proxy->ioType.value == paramHelp::PARAM_IN_OUT) {
            const paramHelp::ParamProxyBasic<double>* proxyBasicDouble =
                  dynamic_cast<const paramHelp::ParamProxyBasic<double>* >(proxy);
            const paramHelp::ParamProxyBasic<int>* proxyBasicInt =
                  dynamic_cast<const paramHelp::ParamProxyBasic<int>* >(proxy);
            if( proxyBasicDouble != NULL) {

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

                    const paramHelp::ParamConstraint<double>* constraints = proxyBasicDouble->constraint;
                    const paramHelp::ParamBilatBounds<double>* bilatBounds =
                          dynamic_cast<const paramHelp::ParamBilatBounds<double>* >(constraints);
                    const paramHelp::ParamLowerBound<double>* lowerBound =
                          dynamic_cast<const paramHelp::ParamLowerBound<double>* >(constraints);
                    const paramHelp::ParamUpperBound<double>* upperBound =
                          dynamic_cast<const paramHelp::ParamUpperBound<double>* >(constraints);

                    if(bilatBounds != NULL)
                        adjustments << "\
    <property name=\"lower\">" << bilatBounds->lowerBound << "</property>\n\
    <property name=\"upper\">" << bilatBounds->upperBound << "</property>" << std::endl;

                    else if(lowerBound != NULL)
                        adjustments << "\
    <property name=\"lower\">" << lowerBound->lowerBound << "</property>\n\
    <property name=\"upper\">" << DEFAULT_UPPER_BOUND << "</property>" << std::endl;

                    else if(upperBound != NULL)
                        adjustments << "\
    <property name=\"lower\">" << DEFAULT_LOWER_BOUND << "</property>\n\
    <property name=\"upper\">" << upperBound->upperBound << "</property>" << std::endl;

                    else
                        adjustments << "\
    <property name=\"lower\">" << DEFAULT_LOWER_BOUND << "</property>\n\
    <property name=\"upper\">" << DEFAULT_UPPER_BOUND << "</property>" << std::endl;

                    adjustments << "\
    <property name=\"step_increment\">0.1</property>\n\
    <property name=\"page_increment\">10</property>\n\
  </object>" << std::endl;

                    main << "\
                      <child>\n\
                        <object class=\"GtkBox\" id=\"hbox_" << proxy->id + displacement << "\">\n\
                          <property name=\"visible\">True</property>\n\
                          <property name=\"can_focus\">False</property>\n\
                          <child>" << std::endl;
                    if(bilatBounds != NULL || lowerBound != NULL || upperBound != NULL)
                        main << "\
                            <object class=\"GtkScale\" id=\"scale_" << proxy->id + displacement << "\">\n\
                              <property name=\"width_request\">200</property>\n\
                              <property name=\"visible\">True</property>\n\
                              <property name=\"can_focus\">False</property>\n\
                              <property name=\"adjustment\">adjustment_" << proxy->id + displacement << "</property>\n\
                              <property name=\"restrict_to_fill_level\">False</property>\n\
                              <property name=\"fill_level\">0</property>\n\
                              <property name=\"round_digits\">2</property>\n\
                              <property name=\"digits\">2</property>\n\
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
                              <property name=\"digits\">2</property>\n\
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
                            else
                            main << "\
                                <object class=\"GtkSpinButton\" id=\"spinbutton_" << proxy->id + displacement << "\">\n\
                                  <property name=\"width_request\">315</property>\n\
                                  <property name=\"visible\">True</property>\n\
                                  <property name=\"can_focus\">True</property>\n\
                                  <property name=\"invisible_char\">•</property>\n\
                                  <property name=\"input_purpose\">digits</property>\n\
                                  <property name=\"adjustment\">adjustment_" << proxy->id + displacement << "</property>\n\
                                  <property name=\"digits\">2</property>\n\
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

            } else if(proxyBasicInt != NULL) {
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

                      const paramHelp::ParamConstraint<int>* constraints = proxyBasicInt->constraint;
                      const paramHelp::ParamBilatBounds<int>* bilatBounds =
                            dynamic_cast<const paramHelp::ParamBilatBounds<int>* >(constraints);
                      const paramHelp::ParamLowerBound<int>* lowerBound =
                            dynamic_cast<const paramHelp::ParamLowerBound<int>* >(constraints);
                      const paramHelp::ParamUpperBound<int>* upperBound =
                            dynamic_cast<const paramHelp::ParamUpperBound<int>* >(constraints);


                      if(bilatBounds != NULL)
                          adjustments << "\
      <property name=\"lower\">" << bilatBounds->lowerBound << "</property>\n\
      <property name=\"upper\">" << bilatBounds->upperBound << "</property>" << std::endl;

                      else if(lowerBound != NULL)
                          adjustments << "\
      <property name=\"lower\">" << lowerBound->lowerBound << "</property>\n\
      <property name=\"upper\">" << DEFAULT_UPPER_BOUND << "</property>" << std::endl;

                      else if(upperBound != NULL)
                          adjustments << "\
      <property name=\"lower\">" << DEFAULT_LOWER_BOUND << "</property>\n\
      <property name=\"upper\">" << upperBound->upperBound << "</property>" << std::endl;

                      else
                          adjustments << "\
      <property name=\"lower\">" << DEFAULT_LOWER_BOUND << "</property>\n\
      <property name=\"upper\">" << DEFAULT_UPPER_BOUND << "</property>" << std::endl;

                     adjustments << "\
     <property name=\"step_increment\">1</property>\n\
     <property name=\"page_increment\">10</property>\n\
   </object>" << std::endl;

                     main << "\
                       <child>\n\
                         <object class=\"GtkBox\" id=\"hbox_" << proxy->id + displacement << "\">\n\
                           <property name=\"visible\">True</property>\n\
                           <property name=\"can_focus\">False</property>\n\
                           <child>" << std::endl;

                     if(bilatBounds != NULL || lowerBound != NULL || upperBound != NULL)
                        main << "\
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
                      else
                         main << "\
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
                  <child>\n\
                    <object class=\"GtkLabel\" id=\"intro_label\">\n\
                      <property name=\"visible\">True</property>\n\
                      <property name=\"can_focus\">False</property>\n\
                      <property name=\"label\" translatable=\"yes\">&lt;b&gt;paramsTuner&lt;/b&gt; allows to change the controller \n\
parameters online.\n\
\n\
You can mouse over the labels to have a \n\
description of every parameter.\n\
Scalar parameters of type double, int and bool\n\
are currently supported.\n\
\n\
&lt;i&gt;Please use the scrollbar on the right instead of the\n\
scrolling wheel on your mouse to avoid changing\n\
the slides by chance.&lt;/i&gt;</property>\n\
                      <property name=\"use_markup\">True</property>\n\
                    </object>\n\
                    <packing>\n\
                      <property name=\"expand\">False</property>\n\
                      <property name=\"fill\">True</property>\n\
                      <property name=\"position\">0</property>\n\
                      <property name=\"padding\">5</property>\n\
                    </packing>\n\
                  </child>\n\                            
                <child>\n\
                  <object class=\"GtkButton\" id=\"button_save\">\n\
                    <property name=\"label\">gtk-save</property>\n\
                    <property name=\"visible\">True</property>\n\
                    <property name=\"can_focus\">False</property>\n\
                    <property name=\"receives_default\">True</property>\n\
                    <property name=\"use_stock\">True</property>\n\
                    <signal name=\"clicked\" handler=\"onSaveButtonClick\" swapped=\"no\"/>\n\
                  </object>\n\
                </child>\n\
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

