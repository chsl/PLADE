/********************************************************************
 * Copyright (C) 2015 Liangliang Nan <liangliang.nan@gmail.com>
 * https://3d.bk.tudelft.nl/liangliang/
 *
 * This file is part of Easy3D. If it is useful in your research/work,
 * I would be grateful if you show your appreciation by citing it:
 * ------------------------------------------------------------------
 *      Liangliang Nan.
 *      Easy3D: a lightweight, easy-to-use, and efficient C++ library
 *      for processing and rendering 3D data.
 *      Journal of Open Source Software, 6(64), 3255, 2021.
 * ------------------------------------------------------------------
 *
 * Easy3D is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License Version 3
 * as published by the Free Software Foundation.
 *
 * Easy3D is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 ********************************************************************/

#include "ply_reader.h"

#include <cstring>
#include <unordered_map>
#include <iostream>
#include <limits>
#include <algorithm>

#include <rply.h>


PlyReader::~PlyReader() {
    for (auto prop: list_properties_)
        delete prop;
    for (auto prop: value_properties_)
        delete prop;
}


bool PlyReader::read(const std::string &file_name, std::vector<Element> &elements) {
    p_ply ply = ply_open(file_name.c_str(), nullptr, 0, nullptr);
    if (!ply) {
        std::cerr << "failed to open ply file: " << file_name << std::endl;
        return false;
    }

    if (!ply_read_header(ply)) {
        std::cerr << "failed to read ply header" << std::endl;
        ply_close(ply);
        return false;
    }

    // setup callbacks
    auto callback_value_property = [](p_ply_argument argument) -> int {
        long instance_index = 0; // the index of the current element instance
        ply_get_argument_element(argument, nullptr, &instance_index);

        ValueProperty *prop_ptr = nullptr;
        ply_get_argument_user_data(argument, (void **) (&prop_ptr), nullptr);

        auto &prop = *prop_ptr;
        prop[instance_index] = ply_get_argument_value(argument);

        return 1;
    };

    auto callback_list_property = [](p_ply_argument argument) -> int {
        long instance_index = 0; // the index of the current element instance
        ply_get_argument_element(argument, nullptr, &instance_index);

        long length = 0, value_index = 0;
        ply_get_argument_property(argument, nullptr, &length, &value_index);
        if (value_index < 0 || value_index >= length) {
            return 1;
        }

        ListProperty *prop_ptr = nullptr;
        ply_get_argument_user_data(argument, (void **) (&prop_ptr), nullptr);

        auto &prop = *prop_ptr;
        auto &entry = prop[instance_index];
        if (entry.empty())
            entry.resize(length);
        entry[value_index] = ply_get_argument_value(argument);

        return 1;
    };

    p_ply_element element = nullptr;
    /* iterate over all elements in input file */
    while ((element = ply_get_next_element(ply, element))) {
        long num_instances = 0;
        const char *element_name = nullptr;
        ply_get_element_info(element, &element_name, &num_instances);
        if (num_instances <= 0)
            continue;

        p_ply_property property = nullptr;
        /* iterate over all properties of current element */
        while ((property = ply_get_next_property(element, property))) {
            const char *property_name = nullptr;
            e_ply_type type, length_type, value_type;
            ply_get_property_info(property, &property_name, &type, &length_type, &value_type);

            // It is possible to save all properties as PLY_LIST of value type double. This allows me to use
            // the same callback function to handle all the properties. But the performance is low. So I handle
            // list properties and value properties separately.
            if (type == PLY_LIST) {
                ListProperty *prop = new ListProperty(element_name, property_name);
                prop->orig_value_type = value_type;
                prop->resize(num_instances);
                list_properties_.push_back(prop);
                if (!ply_set_read_cb(ply, element_name, property_name, callback_list_property, prop, 0)) {
                    std::cerr << "failed to set callback for list property '" << property_name
                               << "' for element '" << element_name << "'" << std::endl;
                    return false;
                }
            } else {
                ValueProperty *prop = new ValueProperty(element_name, property_name);
                prop->orig_value_type = type;
                prop->resize(num_instances);
                value_properties_.push_back(prop);
                if (!ply_set_read_cb(ply, element_name, property_name, callback_value_property, prop, 0)) {
                    std::cerr << "failed to set callback for property '" << property_name << "' for element '"
                               << element_name << "'" << std::endl;
                    return false;
                }
            }

        }
    }

    if (!ply_read(ply)) {
        std::cerr << "error occurred while parsing ply file" << std::endl;
        ply_close(ply);
        return false;
    }

    ply_close(ply);

    // create the elements
    collect_elements(elements);

    // cleaning
    for (auto prop: list_properties_) delete prop;
    list_properties_.clear();
    for (auto prop: value_properties_) delete prop;
    value_properties_.clear();

    return (elements.size() > 0 && elements[0].num_instances > 0);
}


std::size_t PlyReader::num_instances(const std::string &file_name, const std::string &name) {
    p_ply ply = ply_open(file_name.c_str(), nullptr, 0, nullptr);
    if (!ply) {
        std::cerr << "failed to open ply file: " << file_name << std::endl;
        return 0;
    }

    if (!ply_read_header(ply)) {
        std::cerr << "failed to read ply header" << std::endl;
        ply_close(ply);
        return 0;
    }

    p_ply_element element = nullptr;
    /* iterate over all elements in input file */
    while ((element = ply_get_next_element(ply, element))) {
        long num_element = 0;
        const char *element_name = nullptr;
        ply_get_element_info(element, &element_name, &num_element);

        if (!strcmp(element_name, name.c_str())) {
            if (num_element > 0) {
                ply_close(ply);
                return static_cast<std::size_t>(num_element);
            }
        }
    }
    ply_close(ply);
    return 0;
}


namespace details {

    template<typename VT_Input, typename VT_Output>
    inline void convert(const GenericProperty <VT_Input> &input, GenericProperty <VT_Output> &output) {
        output.resize(input.size());
        output.name = input.name;
        for (std::size_t i = 0; i < input.size(); ++i) {
            output[i] = static_cast<VT_Output>(input[i]);
        }
    }

    template<typename VT_Input, typename VT_Output>
    inline void convert(const GenericProperty <std::vector<VT_Input>> &input,
                        GenericProperty <std::vector<VT_Output>> &output) {
        output.resize(input.size());
        output.name = input.name;
        for (std::size_t i = 0; i < input.size(); ++i) {
            const auto &v_in = input[i];
            auto &v_out = output[i];
            v_out.resize(v_in.size());
            for (std::size_t j = 0; j < v_in.size(); ++j)
                v_out[j] = static_cast<VT_Output>(v_in[j]);
        }
    }


    template<typename PropertyT>
    inline bool
    extract_named_property(std::vector<PropertyT> &properties, PropertyT &wanted, const std::string &name) {
        typename std::vector<PropertyT>::iterator it = properties.begin();
        for (; it != properties.end(); ++it) {
            const PropertyT &property = *it;
            if (property.name == name) {
                wanted = property;
                properties.erase(it);
                return true;
            }
        }
        return false;
    }

    template<typename PropertyT>
    inline bool extract_vector_property(std::vector<PropertyT> &properties,
                                        const std::string &x_name, const std::string &y_name,
                                        const std::string &z_name,
                                        Vec3Property &prop) {
        PropertyT x_coords, y_coords, z_coords;
        if (details::extract_named_property(properties, x_coords, x_name) &&
            details::extract_named_property(properties, y_coords, y_name) &&
            details::extract_named_property(properties, z_coords, z_name)) {
            std::size_t num = x_coords.size();
            prop.resize(num);
            for (std::size_t j = 0; j < num; ++j)
                prop[j] = vec3(
                        static_cast<float>(x_coords[j]),
                        static_cast<float>(y_coords[j]),
                        static_cast<float>(z_coords[j])
                );
            return true;
        } else
            return false;
    }

    template<typename PropertyT>
    inline bool extract_vector_property(std::vector<PropertyT> &properties,
                                        const std::string &x_name, const std::string &y_name,
                                        Vec2Property &prop) {
        PropertyT x_coords, y_coords;
        if (details::extract_named_property(properties, x_coords, x_name) &&
            details::extract_named_property(properties, y_coords, y_name)) {
            std::size_t num = x_coords.size();
            prop.resize(num);
            for (std::size_t j = 0; j < num; ++j)
                prop[j] = vec2(
                        static_cast<float>(x_coords[j]),
                        static_cast<float>(y_coords[j])
                );
            return true;
        } else
            return false;
    }

} // namespace details


void PlyReader::collect_elements(std::vector<Element> &elements) const {
    elements.clear();

    // collect all element names and num of instances
    std::unordered_map<std::string, std::size_t> element_sizes;
    for (auto prop: list_properties_) {
        if (element_sizes.find(prop->element_name) == element_sizes.end())
            element_sizes[prop->element_name] = prop->size();
    }
    for (auto prop: value_properties_) {
        if (element_sizes.find(prop->element_name) == element_sizes.end())
            element_sizes[prop->element_name] = prop->size();
    }

    // create all the elements
    for (const auto &element: element_sizes)
        elements.emplace_back(Element(element.first, element.second));

    // a mapping from element name to its pointer
    std::unordered_map<std::string, Element *> name_to_element;
    for (auto &element: elements)
        name_to_element[element.name] = &element;

    for (auto prop: list_properties_) {
        Element *element = name_to_element[prop->element_name];

        e_ply_type type = e_ply_type(prop->orig_value_type);
        if (type == PLY_FLOAT || type == PLY_DOUBLE || type == PLY_FLOAT32 || type == PLY_FLOAT64) {
            FloatListProperty values;
            details::convert(*prop, values);
            element->float_list_properties.push_back(values);
        } else { // must be one of the following integer types:
            //       PLY_INT8, PLY_UINT8, PLY_INT16, PLY_UINT16, PLY_INT32, PLY_UINT32,
            //       PLY_CHAR, PLY_UCHAR, PLY_SHORT, PLY_USHORT, PLY_INT, PLY_UINT
            IntListProperty values;
            details::convert(*prop, values);
            element->int_list_properties.push_back(values);
        }
    }

    for (auto prop: value_properties_) {
        Element *element = name_to_element[prop->element_name];

        e_ply_type type = e_ply_type(prop->orig_value_type);
        if (type == PLY_FLOAT || type == PLY_DOUBLE || type == PLY_FLOAT32 || type == PLY_FLOAT64) {
            FloatProperty values;
            details::convert(*prop, values);
            element->float_properties.push_back(values);
        } else { // must be one of the following integer types:
            //       PLY_INT8, PLY_UINT8, PLY_INT16, PLY_UINT16, PLY_INT32, PLY_UINT32,
            //       PLY_CHAR, PLY_UCHAR, PLY_SHORT, PLY_USHORT, PLY_INT, PLY_UINT
            IntProperty values;
            details::convert(*prop, values);
            element->int_properties.push_back(values);
        }
    }

    // extract some standard vec3 properties, e.g., points, normals, colors, texture coords
    for (auto &element: elements) {
        Vec3Property prop_point("point");
        if (details::extract_vector_property(element.float_properties, "x", "y", "z", prop_point) ||
            details::extract_vector_property(element.float_properties, "X", "Y", "Z", prop_point)) {
            element.vec3_properties.push_back(prop_point);
        }

        Vec2Property prop_texcoord("texcoord");
        if (details::extract_vector_property(element.float_properties, "texcoord_x", "texcoord_y", prop_texcoord)) {
            element.vec2_properties.push_back(prop_texcoord);
        }

        Vec3Property prop_normal("normal");
        if (details::extract_vector_property(element.float_properties, "nx", "ny", "nz", prop_normal)) {
            element.vec3_properties.push_back(prop_normal);
            // check if the normals are normalized
            if (!prop_normal.empty()) {
                const float len = prop_normal[0].length();
                if (std::abs(1.0 - len) > 1e-4)
                    std::cerr << "normals (defined on element '" << element.name << "') not normalized (length of the first normal vector is " << len << ")";
            }
        }

        Vec3Property prop_color("color");
        if (details::extract_vector_property(element.float_properties, "r", "g", "b", prop_color))
            element.vec3_properties.push_back(prop_color);
        else if (details::extract_vector_property(element.int_properties, "red", "green", "blue", prop_color) ||
                 details::extract_vector_property(element.int_properties, "diffuse_red", "diffuse_green",
                                                  "diffuse_blue", prop_color)) {
            for (std::size_t i = 0; i < prop_color.size(); ++i) {
                prop_color[i].x /= 255.0f;
                prop_color[i].y /= 255.0f;
                prop_color[i].z /= 255.0f;
            }
            element.vec3_properties.push_back(prop_color);
        }

        // "alpha" property is stored separately (if exists)
        FloatProperty prop_alpha("alpha");
        if (details::extract_named_property(element.float_properties, prop_alpha, "a"))
            element.float_properties.push_back(prop_alpha);

        else { // might be in Int format
            IntProperty temp("alpha");
            if (details::extract_named_property(element.int_properties, temp, "alpha")) {
                prop_alpha.resize(temp.size());
                for (std::size_t i = 0; i < prop_alpha.size(); ++i)
                    prop_alpha[i] = temp[i] / 255.0f;
                element.float_properties.push_back(prop_alpha);
            }
        }
    }
}